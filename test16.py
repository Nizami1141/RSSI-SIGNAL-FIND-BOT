import time
import math
from collections import deque # Импортируем очередь для истории (как в signal.py)
from picarx import Picarx

# ================= 1. НАСТРОЙКИ (ИЗ ВАШЕГО КОДА) =================
# --- ФИЗИКА СИГНАЛА ---
RSSI_AT_1M = -50        # (A) RSSI на расстоянии 1 метр
PATH_LOSS_EXPONENT = 2.5 # (N) Коэффициент затухания (2.0 - 4.0)

# --- НАСТРОЙКИ ФИЛЬТРАЦИИ (КАК В SIGNAL.PY) ---
WINDOW_SIZE = 10        # Размер окна усреднения (больше = плавнее, но медленнее реакция)

# --- ЦЕЛИ МИССИИ ---
# Робот остановится, когда расчетная дистанция будет от 0.5м до 0.9м
TARGET_DISTANCE_M = 0.7   # Целевая дистанция (0.7 метра)
DIST_TOLERANCE = 0.2      # Допуск +/- 0.2м

# --- МЕХАНИКА ---
SPEED_FAST = 40
SPEED_SLOW = 20
MAX_TURN = 30
OBSTACLE_LIMIT = 25       # См

class RobotBrain:
    def __init__(self):
        self.bot = Picarx()
        
        # Инициализация буфера истории (как в signal.py)
        self.rssi_history = deque(maxlen=WINDOW_SIZE)
        
        self.state = "SEARCH" # Состояния: SEARCH, APPROACH, AVOID, FINISH
        self.last_avg_rssi = -100

    def get_rssi_linux(self):
        """Чтение RSSI из /proc/net/wireless (функция из вашего скриншота)"""
        try:
            with open("/proc/net/wireless", "r") as f:
                lines = f.readlines()
                for line in lines:
                    if "wlan0" in line:
                        parts = line.split()
                        # Индекс [3] обычно level (RSSI). Убираем точку, если есть.
                        rssi = float(parts[3].replace('.', ''))
                        return rssi
        except Exception:
            return None
        return None

    def calculate_distance(self, rssi):
        """
        Формула из вашего скриншота:
        power = (A - rssi) / (10 * n)
        return 10 ** power
        """
        if rssi is None: return 0
        if rssi > -10: return 0.1 # Защита от слишком сильного сигнала
        
        power = (RSSI_AT_1M - rssi) / (10 * PATH_LOSS_EXPONENT)
        return 10 ** power

    def get_smoothed_data(self):
        """
        Читает сигнал, добавляет в историю, считает среднее.
        Возвращает (avg_rssi, distance)
        """
        raw_rssi = self.get_rssi_linux()
        
        if raw_rssi is not None:
            # 1. Добавляем в историю (старые удаляются сами из-за maxlen)
            self.rssi_history.append(raw_rssi)
            
            # 2. Считаем среднее
            if len(self.rssi_history) > 0:
                avg_rssi = sum(self.rssi_history) / len(self.rssi_history)
            else:
                avg_rssi = raw_rssi
                
            # 3. Считаем дистанцию по СРЕДНЕМУ
            dist = self.calculate_distance(avg_rssi)
            return avg_rssi, dist
        
        return None, None

    def scan_for_direction(self):
        """Останавливается и ищет лучший сигнал (влево/прямо/вправо)"""
        print("   👀 Сканирую направление сигнала...")
        self.bot.stop()
        best_rssi = -999
        best_angle = 0
        
        # Проверяем углы: -30 (лево), 0 (центр), 30 (право)
        for angle in [-30, 0, 30]:
            self.bot.set_dir_servo_angle(angle)
            time.sleep(0.5)
            
            # Делаем несколько быстрых замеров для этого угла
            temp_history = []
            for _ in range(5): # 5 замеров
                r = self.get_rssi_linux()
                if r: temp_history.append(r)
                time.sleep(0.1)
            
            if temp_history:
                avg = sum(temp_history) / len(temp_history)
                if avg > best_rssi:
                    best_rssi = avg
                    best_angle = angle
        
        print(f"   👉 Лучшее направление: {best_angle} град. (RSSI: {best_rssi:.1f})")
        return best_angle

    def run(self):
        print(f"--- 🤖 ЗАПУСК РОБОТА. Цель: {TARGET_DISTANCE_M}м (+/- {DIST_TOLERANCE}) ---")
        print(f"--- Калибровка: A={RSSI_AT_1M}, N={PATH_LOSS_EXPONENT}, Window={WINDOW_SIZE} ---")
        
        try:
            while True:
                # 1. ПОЛУЧАЕМ СГЛАЖЕННЫЕ ДАННЫЕ
                avg_rssi, estimated_meters = self.get_smoothed_data()
                obstacle_dist = self.bot.ultrasonic.read()

                # Если данных нет, используем последние известные
                if avg_rssi is None:
                    if self.last_avg_rssi != -100:
                        avg_rssi = self.last_avg_rssi
                        estimated_meters = self.calculate_distance(avg_rssi)
                    else:
                        continue # Ждем первый сигнал
                else:
                    self.last_avg_rssi = avg_rssi

                # Логирование
                print(f"[{self.state}] RSSI: {avg_rssi:.1f} dBm | Дист: {estimated_meters:.2f} м")

                # 2. ПРОВЕРКА ПРЕПЯТСТВИЙ
                if obstacle_dist > 0 and obstacle_dist < OBSTACLE_LIMIT:
                    print(f"⛔ ПРЕПЯТСТВИЕ ({obstacle_dist}см)! Отъезжаю...")
                    self.state = "AVOID"

                # 3. ЛОГИКА ДВИЖЕНИЯ
                
                # --- A: Объезд препятствия ---
                if self.state == "AVOID":
                    self.bot.backward(SPEED_SLOW)
                    time.sleep(1.0)
                    self.bot.set_dir_servo_angle(MAX_TURN) # Поворот направо
                    self.bot.forward(SPEED_SLOW)
                    time.sleep(0.8)
                    self.state = "SEARCH"
                    # УДАЛЕНО: self.rssi_history.clear() 
                    # Мы НЕ стираем историю, чтобы избежать скачков дистанции

                # --- B: ФИНИШ (Мы в зоне цели) ---
                elif (TARGET_DISTANCE_M - DIST_TOLERANCE) <= estimated_meters <= (TARGET_DISTANCE_M + DIST_TOLERANCE):
                    self.bot.stop()
                    self.bot.set_dir_servo_angle(0)
                    print(f"\n🎉 ЦЕЛЬ НАЙДЕНА! Дистанция {estimated_meters:.2f}м (Цель была {TARGET_DISTANCE_M}м)")
                    break

                # --- C: ПОИСК (Далеко) ---
                elif self.state == "SEARCH":
                    if estimated_meters < 4.0:
                        # Если мы ближе 4 метров, переходим в точный режим
                        self.state = "APPROACH"
                    else:
                        # Если далеко - просто едем
                        self.bot.forward(SPEED_FAST)
                        self.bot.set_dir_servo_angle(0)

                # --- D: СБЛИЖЕНИЕ (Точно) ---
                elif self.state == "APPROACH":
                    # Если мы пролетели мимо и удаляемся (дистанция растет) или сигнал слишком слабый
                    if estimated_meters > (TARGET_DISTANCE_M + 1.5):
                        # Потеряли цель или уехали не туда -> сканируем
                        angle = self.scan_for_direction()
                        self.bot.set_dir_servo_angle(angle)
                        self.bot.forward(SPEED_SLOW)
                        time.sleep(1.0)
                        # УДАЛЕНО: self.rssi_history.clear()
                    else:
                        # Плавно едем к цели
                        self.bot.set_dir_servo_angle(0)
                        self.bot.forward(SPEED_SLOW)

                time.sleep(0.1) # Небольшая пауза цикла

        except KeyboardInterrupt:
            print("\n🛑 Остановка пользователем.")
        finally:
            self.bot.stop()

if __name__ == "__main__":
    brain = RobotBrain()
    brain.run()
