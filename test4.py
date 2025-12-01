import time
from collections import deque
from picarx import Picarx

# --- ИНИЦИАЛИЗАЦИЯ ---
px = Picarx()

# --- ЗОНЫ ПОИСКА ---
FINISH_THRESHOLD = -46     # Победа!
PRECISION_ZONE_START = -50 # Начало зоны точного поиска (-50...-47)

# --- ПАРАМЕТРЫ ДВИЖЕНИЯ ---
FAST_SPEED = 50      # Скорость вдалеке
FAST_TURN = 35       # Резкий поворот вдалеке

SLOW_SPEED = 20      # Скорость вблизи (точная наводка)
SLOW_TURN = 15       # Плавный поворот вблизи

# --- СИСТЕМНЫЕ НАСТРОЙКИ ---
TARGET_RSSI = -40    # Цель для Cost Function
WINDOW_SIZE = 10     # Сглаживание
rssi_history = deque(maxlen=WINDOW_SIZE)
prev_cost = None
OBSTACLE_LIMIT = 20  # См для Ultrasonic

# ================= ФУНКЦИИ =================

def get_rssi_linux():
    try:
        with open("/proc/net/wireless", "r") as f:
            lines = f.readlines()
            for line in lines:
                if "wlan0" in line:
                    parts = line.split()
                    rssi = float(parts[3].replace('.', ''))
                    return rssi
    except Exception:
        return None

def calculate_cost(current_rssi, target):
    return abs(target - current_rssi)

def stop_robot():
    px.forward(0)
    px.set_dir_servo_angle(0)

# ================= ЦИКЛ =================

print(f"--- PicarX: Smart Search ---")
print(f"1. Быстрый подход (< {PRECISION_ZONE_START} dBm)")
print(f"2. Точная наводка ({PRECISION_ZONE_START} ... {FINISH_THRESHOLD} dBm)")
print(f"3. Финиш (>= {FINISH_THRESHOLD} dBm)")
time.sleep(2)

try:
    while True:
        # --- 1. СЕНСОРЫ ---
        physical_dist = px.ultrasonic.read() 
        raw_rssi = get_rssi_linux()

        # --- 2. БЕЗОПАСНОСТЬ ---
        if physical_dist > 0 and physical_dist < OBSTACLE_LIMIT:
            print(f"⛔ ПРЕПЯТСТВИЕ ({physical_dist}см) -> ОТЪЕЗД")
            px.stop()
            time.sleep(0.2)
            px.set_dir_servo_angle(-30) 
            px.backward(30)             
            time.sleep(1.0)             
            px.stop()
            rssi_history.clear()
            prev_cost = None
            continue 

        # --- 3. ЛОГИКА ---
        if raw_rssi is not None:
            rssi_history.append(raw_rssi)
            
            if len(rssi_history) < WINDOW_SIZE:
                time.sleep(0.1)
                continue

            avg_rssi = sum(rssi_history) / len(rssi_history)
            
            # === ПРОВЕРКА: ГДЕ МЫ? ===
            
            # А. ФИНИШ
            if avg_rssi >= FINISH_THRESHOLD:
                print("\n" + "="*40)
                print(f"🏁 ПРИБЫЛИ! Сигнал: {avg_rssi:.1f} dBm")
                print("="*40 + "\n")
                stop_robot()
                break 

            # Б. ОПРЕДЕЛЕНИЕ РЕЖИМА (БЫСТРО или ТОЧНО)
            if avg_rssi >= PRECISION_ZONE_START:
                # Мы в зоне от -50 до -47
                mode = "🎯 ТОЧНО"
                current_speed = SLOW_SPEED
                current_turn = SLOW_TURN
            else:
                # Мы далеко (хуже -50)
                mode = "🚀 БЫСТРО"
                current_speed = FAST_SPEED
                current_turn = FAST_TURN

            # В. COST FUNCTION (ТЕПЛЕЕТ/ХОЛОДАЕТ)
            current_cost = calculate_cost(avg_rssi, TARGET_RSSI)

            if prev_cost is None:
                prev_cost = current_cost
                px.forward(current_speed)
                continue

            delta = current_cost - prev_cost 
            
            if delta <= 0:
                # УЛУЧШЕНИЕ -> Едем прямо
                print(f"[{mode}] {avg_rssi:.1f} dBm | ✅ ПРЯМО (Spd: {current_speed})")
                px.set_dir_servo_angle(0)
                px.forward(current_speed) 
            else:
                # УХУДШЕНИЕ -> Поворачиваем
                print(f"[{mode}] {avg_rssi:.1f} dBm | ❄️ ИЩУ (Ang: {current_turn})")
                px.set_dir_servo_angle(current_turn) 
                px.forward(current_speed) # Продолжаем ехать, но поворачивая

            prev_cost = current_cost
        else:
            px.stop()

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nСтоп.")
    stop_robot()
