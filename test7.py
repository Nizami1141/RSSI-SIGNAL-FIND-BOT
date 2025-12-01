import time
import random
from collections import deque
from picarx import Picarx

# --- ИНИЦИАЛИЗАЦИЯ ---
px = Picarx()

# --- НАСТРОЙКИ КАЛИБРОВКИ (ИСПРАВЛЕНО) ---
# A = Уровень сигнала на расстоянии 1 метр. 
# Исправлено с -50 на -41, чтобы на 3 метрах показывало правильно.
A = -41  
N = 2.5   # Коэффициент затухания (офис/квартира)

# --- НАСТРОЙКИ ПОИСКА ---
FINISH_THRESHOLD = -46     # Если сигнал >= -46, стоп (мы у цели ~1.5 метра)
PRECISION_ZONE = -52       # Зона точного поиска (снижаем скорость)
RSSI_TOLERANCE = 1.5       # Игнорируем шум менее 1.5 dBm

# --- ПАРАМЕТРЫ ДВИЖЕНИЯ ---
FAST_SPEED = 50
SLOW_SPEED = 25
TURN_ANGLE = 35

# --- СИСТЕМНЫЕ ПЕРЕМЕННЫЕ ---
WINDOW_SIZE = 8            
rssi_history = deque(maxlen=WINDOW_SIZE)
last_avg_rssi = None       

# Направление поиска: 1 = Вправо, -1 = Влево
search_direction = 1       

# --- БЕЗОПАСНОСТЬ ---
OBSTACLE_LIMIT = 20        # См

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

def calculate_distance(rssi):
    """Формула перевода dBm в Метры"""
    if rssi is None: return 0
    power = (A - rssi) / (10 * N)
    return 10 ** power

def stop_robot():
    px.forward(0)
    px.set_dir_servo_angle(0)

def smart_backup():
    """Умный отъезд от препятствия"""
    print("⛔ ПРЕПЯТСТВИЕ -> ОТЪЕЗД")
    px.stop()
    time.sleep(0.2)
    
    # Отъезжаем назад, поворачивая в обратную сторону от текущего поиска
    backup_angle = -35 if search_direction == 1 else 35
    px.set_dir_servo_angle(backup_angle)
    
    px.backward(40)
    time.sleep(1.0) 
    px.stop()
    
    # Сбрасываем историю, так как позиция изменилась
    rssi_history.clear()
    global last_avg_rssi
    last_avg_rssi = None

# ================= ОСНОВНОЙ ЦИКЛ =================

print(f"--- Wi-Fi Seeker v3.1 (Calibrated) ---")
print(f"Цель: >= {FINISH_THRESHOLD} dBm")
print(f"Калибровка: A={A}, N={N}")
stop_robot()
time.sleep(2)

try:
    while True:
        # 1. ЧТЕНИЕ ДАТЧИКОВ
        dist_obstacle = px.ultrasonic.read()
        raw_rssi = get_rssi_linux()

        # 2. БЕЗОПАСНОСТЬ (Приоритет)
        if dist_obstacle > 0 and dist_obstacle < OBSTACLE_LIMIT:
            smart_backup()
            continue

        # 3. ОБРАБОТКА WI-FI
        if raw_rssi is not None:
            rssi_history.append(raw_rssi)
            
            # Ждем заполнения буфера для точности
            if len(rssi_history) < WINDOW_SIZE:
                print(f"Сбор данных... {len(rssi_history)}/{WINDOW_SIZE}")
                time.sleep(0.1)
                continue

            curr_rssi = sum(rssi_history) / len(rssi_history)
            
            # Расчет дистанции для вывода на экран
            estimated_dist = calculate_distance(curr_rssi)

            # --- ПРОВЕРКА ФИНИША ---
            if curr_rssi >= FINISH_THRESHOLD:
                print("\n" + "="*40)
                print(f"🏆 ФИНИШ! Сигнал: {curr_rssi:.1f} dBm")
                print(f"📍 Дистанция: {estimated_dist:.2f} м")
                print("="*40 + "\n")
                stop_robot()
                break

            # --- ВЫБОР СКОРОСТИ ---
            speed = SLOW_SPEED if curr_rssi >= PRECISION_ZONE else FAST_SPEED
            
            # --- ЛОГИКА НАВИГАЦИИ ---
            if last_avg_rssi is None:
                last_avg_rssi = curr_rssi
                px.set_dir_servo_angle(0)
                px.forward(speed)
                continue

            delta = curr_rssi - last_avg_rssi

            # Вывод статуса
            print(f"RSSI: {curr_rssi:.1f} dBm | Dist: {estimated_dist:.2f}m | ", end="")

            if delta > RSSI_TOLERANCE:
                # --- СИГНАЛ РАСТЕТ -> ПРЯМО ---
                print(f"✅ ЛУЧШЕ (+{delta:.1f}) -> ПРЯМО")
                px.set_dir_servo_angle(0)
                px.forward(speed)

            elif delta < -RSSI_TOLERANCE:
                # --- СИГНАЛ ПАДАЕТ -> ПОВОРОТ ---
                print(f"❄️ ХУЖЕ ({delta:.1f}) -> ", end="")

                # Если сигнал падает очень резко, меняем направление поиска сразу
                if delta < -3.0:
                     search_direction *= -1
                     print("РАЗВОРОТ! ", end="")
                
                angle = TURN_ANGLE * search_direction
                print(f"ПОВОРОТ {'ВПРАВО' if search_direction==1 else 'ВЛЕВО'}")
                
                px.set_dir_servo_angle(angle)
                px.forward(speed)
                
                # Подготовка смены направления на следующий раз, если не поможет
                search_direction *= -1 

            else:
                # --- ШУМ -> ЕДЕМ КАК ЕХАЛИ ---
                print(f"~ Стабильно -> ПРЯМО")
                px.forward(speed)

            last_avg_rssi = curr_rssi

        else:
            px.stop()
            print("Ошибка Wi-Fi адаптера")

        time.sleep(0.15) 

except KeyboardInterrupt:
    print("\nОстановка.")
    stop_robot()
