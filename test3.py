import time
from collections import deque
from picarx import Picarx

# --- ИНИЦИАЛИЗАЦИЯ РОБОТА ---
px = Picarx()

# --- НАСТРОЙКИ WI-FI (ПОИСК) ---
TARGET_RSSI = -40      # Идеальная цель (для математики Cost function)
STOP_THRESHOLD = -46   # ПОРОГ ОСТАНОВКИ: Если сигнал >= -46, мы приехали!

WINDOW_SIZE = 10       
rssi_history = deque(maxlen=WINDOW_SIZE)
prev_cost = None

# --- НАСТРОЙКИ ULTRASONIC (БЕЗОПАСНОСТЬ) ---
OBSTACLE_LIMIT = 20    # См. Остановка перед препятствием

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

# ================= ОСНОВНОЙ ЦИКЛ =================

print(f"--- PicarX: Поиск Wi-Fi (Цель: {STOP_THRESHOLD} dBm) ---")
time.sleep(2)

try:
    while True:
        # --- 1. ЧИТАЕМ СЕНСОРЫ ---
        physical_dist = px.ultrasonic.read() 
        raw_rssi = get_rssi_linux()

        # --- 2. БЕЗОПАСНОСТЬ (ПРИОРИТЕТ №1) ---
        if physical_dist > 0 and physical_dist < OBSTACLE_LIMIT:
            print(f"⛔ ПРЕПЯТСТВИЕ! ({physical_dist} см) -> ОТЪЕЗД")
            px.stop()
            time.sleep(0.2)
            px.set_dir_servo_angle(-30) 
            px.backward(30)             
            time.sleep(1.0)             
            px.stop()
            rssi_history.clear()
            prev_cost = None
            continue 

        # --- 3. ЛОГИКА ПОИСКА WI-FI ---
        if raw_rssi is not None:
            rssi_history.append(raw_rssi)
            
            if len(rssi_history) < WINDOW_SIZE:
                print(f"Сбор данных... {len(rssi_history)}/{WINDOW_SIZE}")
                time.sleep(0.1)
                continue

            avg_rssi = sum(rssi_history) / len(rssi_history)
            
            # === НОВАЯ ПРОВЕРКА: МЫ ПРИЕХАЛИ? ===
            if avg_rssi >= STOP_THRESHOLD:
                print("\n" + "="*40)
                print(f"🎉 ЦЕЛЬ НАЙДЕНА! Сигнал: {avg_rssi:.1f} dBm")
                print("="*40 + "\n")
                stop_robot()
                break # Выход из цикла (программа завершится)

            # Расчет Cost Function
            current_cost = calculate_cost(avg_rssi, TARGET_RSSI)

            if prev_cost is None:
                prev_cost = current_cost
                px.set_dir_servo_angle(0)
                px.forward(30)
                continue

            delta = current_cost - prev_cost 
            
            # --- УПРАВЛЕНИЕ ---
            if delta <= 0:
                # Сигнал улучшается -> Едем ПРЯМО
                print(f"✅ ТЕПЛЕЕТ ({avg_rssi:.1f} dBm) -> ПРЯМО")
                px.set_dir_servo_angle(0)
                px.forward(50) 
            else:
                # Сигнал ухудшается -> ПОВОРАЧИВАЕМ
                print(f"❄️ ХОЛОДАЕТ ({avg_rssi:.1f} dBm) -> ИЩУ")
                px.set_dir_servo_angle(35) 
                px.forward(40) 

            prev_cost = current_cost
        else:
            print("Ошибка Wi-Fi адаптера")
            px.stop()

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nСтоп.")
    stop_robot()
