import time
from collections import deque
from picarx import Picarx  # Библиотека PicarX

# --- ИНИЦИАЛИЗАЦИЯ РОБОТА ---
px = Picarx()

# --- НАСТРОЙКИ WI-FI (ПОИСК) ---
TARGET_RSSI = -40   # Цель (dBm): хотим получить такой уровень сигнала
WINDOW_SIZE = 10    # Сглаживание сигнала
rssi_history = deque(maxlen=WINDOW_SIZE)
prev_cost = None    # Для хранения предыдущей ошибки

# --- НАСТРОЙКИ ULTRASONIC (БЕЗОПАСНОСТЬ) ---
OBSTACLE_LIMIT = 20 # См. Если препятствие ближе 20 см - СТОП/ОБЪЕЗД.

# ================= ФУНКЦИИ =================

def get_rssi_linux():
    """Чтение уровня Wi-Fi сигнала из системы"""
    try:
        with open("/proc/net/wireless", "r") as f:
            lines = f.readlines()
            for line in lines:
                if "wlan0" in line:
                    parts = line.split()
                    # Обычно 4-я колонка, удаляем точки
                    rssi = float(parts[3].replace('.', ''))
                    return rssi
    except Exception:
        return None

def calculate_cost(current_rssi, target):
    """
    Функция стоимости (Cost Function).
    Показывает, насколько мы далеки от идеального сигнала.
    """
    return abs(target - current_rssi)

def stop_robot():
    px.forward(0)
    px.set_dir_servo_angle(0)

# ================= ОСНОВНОЙ ЦИКЛ =================

print("--- PicarX: Wi-Fi Seeker + Obstacle Avoidance ---")
print("Запуск через 2 секунды...")
time.sleep(2)

try:
    while True:
        # --- 1. ЧИТАЕМ СЕНСОРЫ ---
        
        # А. Физическое препятствие (Ultrasonic)
        physical_dist = px.ultrasonic.read() 
        
        # Б. Уровень сигнала (Wi-Fi)
        raw_rssi = get_rssi_linux()

        # --- 2. ЛОГИКА БЕЗОПАСНОСТИ (ПРИОРИТЕТ №1) ---
        
        # Если датчик видит стену ближе OBSTACLE_LIMIT см (и значение > 0, т.к. 0 иногда глюк)
        if physical_dist > 0 and physical_dist < OBSTACLE_LIMIT:
            print(f"⛔ ПРЕПЯТСТВИЕ! Дистанция: {physical_dist} см")
            
            # Реакция на препятствие:
            px.stop()
            time.sleep(0.2)
            px.set_dir_servo_angle(-30) # Поворачиваем колеса влево
            px.backward(30)             # Отъезжаем назад
            time.sleep(1.0)             # В течение секунды
            px.stop()
            
            # Сбрасываем историю Wi-Fi, так как мы сместились
            rssi_history.clear()
            prev_cost = None
            continue # Пропускаем остаток цикла, начинаем замер заново

        # --- 3. ЛОГИКА ПОИСКА WI-FI (ЕСЛИ ПУТЬ ЧИСТ) ---
        
        if raw_rssi is not None:
            # Сглаживаем Wi-Fi
            rssi_history.append(raw_rssi)
            
            # Ждем накопления буфера, чтобы не дергаться
            if len(rssi_history) < WINDOW_SIZE:
                print(f"Набор данных... {len(rssi_history)}/{WINDOW_SIZE}")
                time.sleep(0.1)
                continue

            avg_rssi = sum(rssi_history) / len(rssi_history)
            
            # Считаем Cost Function
            current_cost = calculate_cost(avg_rssi, TARGET_RSSI)

            # Если это первый замер, просто запоминаем и едем прямо
            if prev_cost is None:
                prev_cost = current_cost
                px.set_dir_servo_angle(0)
                px.forward(30)
                print("Калибровка... Едем прямо.")
                continue

            # --- ГРАДИЕНТНЫЙ СПУСК (РЕАКЦИЯ МОТОРОВ) ---
            
            delta = current_cost - prev_cost # Изменение ситуации
            
            # PicarX поворачивает сервоприводом (set_dir_servo_angle).
            # Угол 0 = Прямо. Угол > 0 = Вправо/Влево (зависит от сборки).
            
            if delta <= 0:
                # Cost падает (или такой же) -> Сигнал улучшается -> Едем ПРЯМО
                # Также можно чуть ускориться, если мы уверены
                
                print(f"✅ ТЕПЛЕЕТ (Cost {current_cost:.1f}) -> ПРЯМО")
                px.set_dir_servo_angle(0) # Колеса прямо
                px.forward(50)            # Газ

            else:
                # Cost растет -> Сигнал ухудшается (Холоднеет) -> ПОВОРАЧИВАЕМ
                # Мы начинаем искать направление. Повернем колеса.
                
                print(f"❄️ ХОЛОДАЕТ (Cost {current_cost:.1f}) -> ПОВОРОТ")
                
                # Поворачиваем колеса (например, на 35 градусов)
                # Чтобы робот ехал дугой и искал сигнал
                px.set_dir_servo_angle(35) 
                px.forward(40) # Чуть сбавляем скорость на повороте

            # Запоминаем текущую стоимость как "прошлую" для следующего шага
            prev_cost = current_cost
            
            print(f"   [Sensors] Ultra: {physical_dist}cm | Wi-Fi: {avg_rssi:.1f}dBm")

        else:
            print("Wi-Fi адаптер не найден или ошибка чтения.")
            px.stop()

        time.sleep(0.1) # Частота обновления цикла

except KeyboardInterrupt:
    print("\nОстановка робота...")
    stop_robot()
