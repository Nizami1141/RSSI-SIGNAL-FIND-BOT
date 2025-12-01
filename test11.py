import time
import os
import math
from collections import deque
from picarx import Picarx

# ================= КОНФИГУРАЦИЯ =================
TARGET_RSSI = -40        # Цель (dBm)
APPROACH_THRESHOLD = -65 # Порог начала сближения
PROCESS_NOISE_Q = 0.05   # Шум процесса (фильтр Калмана)
MEASUREMENT_NOISE_R = 2.0 # Шум измерений
MAX_STEER_ANGLE = 35     # Максимальный угол поворота
OBSTACLE_DIST_CM = 25    # Дистанция до препятствия
SPEED_SEARCH = 40
SPEED_APPROACH = 30

# ================= КЛАССЫ =================

class RSSIKalmanFilter:
    def __init__(self, R, Q, initial_value=-70):
        self.R = R
        self.Q = Q
        self.x = initial_value
        self.P = 1.0

    def filter(self, measurement):
        if measurement is None:
            return self.x
        
        # Предикция
        p_pred = self.P + self.Q
        
        # Обновление
        K = p_pred / (p_pred + self.R)
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * p_pred
        return self.x

class WiFiSensor:
    def __init__(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def get_rssi(self):
        try:
            with open(self.filepath, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if self.interface in line:
                        parts = line.split()
                        # Индекс 3 - это level (RSSI).
                        # Пример parts: ['wlan0:', '0000', '70.', '-51.',...]
                        if len(parts) >= 4:
                            # ИСПРАВЛЕНО: parts[1] без точки перед скобкой
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
        
        self.center_head()

    def center_head(self):
        self.px.set_camera_servo1_angle(0)
        self.px.set_camera_servo2_angle(0)

    def scan_surroundings(self):
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
            return -1 # Влево свободнее
        else:
            return 1  # Вправо свободнее

    def run(self):
        print(f"--- 🚀 ЗАПУСК PICARX (FIXED) | Цель: {TARGET_RSSI} dBm ---")
        
        try:
            while True:
                # 1. Чтение данных
                raw_rssi = self.wifi.get_rssi()
                dist = self.px.ultrasonic.read()
                
                # Фильтрация
                if raw_rssi is not None:
                    rssi = self.kf.filter(raw_rssi)
                else:
                    rssi = self.last_rssi

                # Логирование (редко)
                if time.time() % 0.5 < 0.05:
                    print(f"[{self.state}] RSSI: {rssi:.1f} | Dist: {dist}")

                # 2. Безопасность
                if dist > 0 and dist < OBSTACLE_DIST_CM and self.state!= "FINISH":
                    print(f"⛔ ПРЕПЯТСТВИЕ ({dist} см)")
                    self.state = "AVOID"

                # 3. Логика состояний
                if self.state == "AVOID":
                    direction = self.scan_surroundings()
                    
                    self.px.set_dir_servo_angle(0)
                    self.px.backward(SPEED_SEARCH)
                    time.sleep(0.8)
                    
                    turn_angle = direction * MAX_STEER_ANGLE
                    self.px.set_dir_servo_angle(turn_angle)
                    self.px.forward(SPEED_SEARCH)
                    time.sleep(0.5)
                    
                    self.last_rssi = rssi 
                    self.state = "SEARCH" 

                elif self.state == "SEARCH":
                    if rssi > APPROACH_THRESHOLD:
                        print(f"✅ Захват сигнала: {rssi:.1f}")
                        self.state = "APPROACH"
                        continue

                    # Спираль
                    self.px.forward(SPEED_SEARCH)
                    self.px.set_dir_servo_angle(int(self.spiral_angle))
                    
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
                        # Сигнал растет -> Прямо
                        self.px.set_dir_servo_angle(0)
                        self.px.forward(SPEED_APPROACH)
                    else:
                        # Сигнал падает -> Рыскание
                        self.px.set_dir_servo_angle(20) 
                        self.px.forward(SPEED_APPROACH)
                        
                elif self.state == "FINISH":
                    self.px.stop()
                    self.px.set_dir_servo_angle(0)
                    print(f"🎉 ИСТОЧНИК НАЙДЕН! RSSI: {rssi:.2f}")
                    break

                self.last_rssi = rssi
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nСтоп.")
        except Exception as e:
            print(f"\nОшибка: {e}")
        finally:
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            self.center_head()

if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
