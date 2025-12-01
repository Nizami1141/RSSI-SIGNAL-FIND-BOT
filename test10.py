import time
import os
import math
from collections import deque
from picarx import Picarx

# ================= КОНФИГУРАЦИЯ =================
# Настройки WiFi
TARGET_RSSI = -40        # Целевой уровень сигнала (мы у источника)
APPROACH_THRESHOLD = -65 # Уровень, при котором переходим от поиска к сближению

# Настройки Фильтра Калмана
PROCESS_NOISE_Q = 0.05   # Параметр изменения реального сигнала
MEASUREMENT_NOISE_R = 2.0 # Параметр шума измерений

# Настройки Робота
MAX_STEER_ANGLE = 35     # Предел угла поворота (защита сервопривода)
OBSTACLE_DIST_CM = 25    # Дистанция остановки перед препятствием
SPEED_SEARCH = 40        # Скорость при поиске
SPEED_APPROACH = 30      # Скорость при точном наведении

# ================= КЛАССЫ =================

class RSSIKalmanFilter:
    """
    Одномерный фильтр Калмана для сглаживания шумного WiFi сигнала.
    """
    def __init__(self, R, Q, initial_value=-70):
        self.R = R  # Шум измерений
        self.Q = Q  # Шум процесса
        self.x = initial_value
        self.P = 1.0

    def filter(self, measurement):
        if measurement is None:
            return self.x
        
        # 1. Предикция
        p_pred = self.P + self.Q
        
        # 2. Обновление
        K = p_pred / (p_pred + self.R)
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * p_pred
        return self.x

class WiFiSensor:
    """
    Класс для быстрого чтения RSSI напрямую из ядра Linux.
    """
    def __init__(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def get_rssi(self):
        try:
            with open(self.filepath, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if self.interface in line:
                        # Пример строки: " wlan0: 0000   70.  -51.  -256..."
                        parts = line.split()
                        
                        # Индексы после split():
                        # 0: wlan0:
                        # 1: 0000 (status)
                        # 2: 70. (link quality)
                        # 3: -51. (RSSI level)
                        
                        if len(parts) >= 4:
                            # Исправленная строка: убрана точка перед скобкой
                            rssi_str = parts.[1]replace('.', '')
                            return float(rssi_str)
        except Exception:
            return None
        return None

class NavigationController:
    def __init__(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        self.kf = RSSIKalmanFilter(R=MEASUREMENT_NOISE_R, Q=PROCESS_NOISE_Q)
        
        self.state = "SEARCH"
        self.last_rssi = -100
        self.start_time = time.time()
        
        self.spiral_angle = -MAX_STEER_ANGLE
        
        # Сброс позиции головы
        self.center_head()

    def center_head(self):
        self.px.set_camera_servo1_angle(0)
        self.px.set_camera_servo2_angle(0)

    def scan_surroundings(self):
        """Сканирование головой влево-вправо при препятствии"""
        self.px.stop()
        time.sleep(0.2)
        
        # Влево
        self.px.set_camera_servo1_angle(-45)
        time.sleep(0.3)
        dist_left = self.px.ultrasonic.read()
        
        # Вправо
        self.px.set_camera_servo1_angle(45)
        time.sleep(0.3)
        dist_right = self.px.ultrasonic.read()
        
        self.center_head()
        time.sleep(0.2)
        
        print(f"👀 Сканирование: Слева={dist_left}см, Справа={dist_right}см")
        if dist_left > dist_right:
            return -1 # Ехать влево
        else:
            return 1  # Ехать вправо

    def run(self):
        print("--- 🚀 ЗАПУСК НАВИГАЦИИ PICARX ---")
        print(f"Цель: {TARGET_RSSI} dBm")
        
        try:
            while True:
                # Чтение сенсоров
                raw_rssi = self.wifi.get_rssi()
                dist = self.px.ultrasonic.read()
                
                # Фильтрация RSSI
                if raw_rssi is not None:
                    rssi = self.kf.filter(raw_rssi)
                else:
                    rssi = self.last_rssi

                # Логирование (раз в 0.5 сек)
                if time.time() % 0.5 < 0.05:
                    print(f"[{self.state}] RSSI: {rssi:.2f} (Raw: {raw_rssi}) | Dist: {dist}")

                # --- БЕЗОПАСНОСТЬ ---
                # Если препятствие ближе 25см и мы еще не приехали
                if dist > 0 and dist < OBSTACLE_DIST_CM and self.state!= "FINISH":
                    print(f"⛔ ПРЕПЯТСТВИЕ ({dist} см)!")
                    self.state = "AVOID"

                # --- МАШИНА СОСТОЯНИЙ ---
                if self.state == "AVOID":
                    direction = self.scan_surroundings()
                    
                    # Отъезд назад
                    self.px.set_dir_servo_angle(0)
                    self.px.backward(SPEED_SEARCH)
                    time.sleep(0.8)
                    
                    # Поворот
                    turn_angle = direction * MAX_STEER_ANGLE
                    self.px.set_dir_servo_angle(turn_angle)
                    self.px.forward(SPEED_SEARCH)
                    time.sleep(0.5)
                    
                    self.last_rssi = rssi 
                    self.state = "SEARCH" 

                elif self.state == "SEARCH":
                    if rssi > APPROACH_THRESHOLD:
                        print(f"✅ Сигнал захвачен ({rssi:.1f}). СБЛИЖЕНИЕ.")
                        self.state = "APPROACH"
                        continue

                    # Спиральное движение
                    self.px.forward(SPEED_SEARCH)
                    self.px.set_dir_servo_angle(int(self.spiral_angle))
                    
                    # Уменьшаем угол поворота (спираль раскручивается)
                    if self.spiral_angle < 0:
                        self.spiral_angle += 0.05
                    else:
                        self.spiral_angle = -MAX_STEER_ANGLE 

                elif self.state == "APPROACH":
                    if rssi >= TARGET_RSSI:
                        self.state = "FINISH"
                        continue
                    
                    delta = rssi - self.last_rssi
                    
                    if delta >= 0:
                        # Сигнал растет - едем прямо
                        self.px.set_dir_servo_angle(0)
                        self.px.forward(SPEED_APPROACH)
                    else:
                        # Сигнал падает - немного поворачиваем (рыскание)
                        self.px.set_dir_servo_angle(20) 
                        self.px.forward(SPEED_APPROACH)
                        
                elif self.state == "FINISH":
                    self.px.stop()
                    self.px.set_dir_servo_angle(0)
                    print(f"🎉 ИСТОЧНИК НАЙДЕН! RSSI: {rssi:.2f}")
                    break

                self.last_rssi = rssi
                time.sleep(0.05) # Небольшая задержка цикла

        except KeyboardInterrupt:
            print("\nОстановка...")
        except Exception as e:
            print(f"\nОшибка выполнения: {e}")
        finally:
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            self.center_head()

if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
