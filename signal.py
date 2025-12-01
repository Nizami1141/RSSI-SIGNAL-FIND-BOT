import time
from collections import deque  # Импортируем очередь для хранения истории

def get_rssi_linux():
    try:
        with open("/proc/net/wireless", "r") as f:
            lines = f.readlines()
            for line in lines:
                if "wlan0" in line:
                    parts = line.split()
                    # Индекс [3] зависит от форматирования, здесь предполагается, 
                    # что уровень сигнала находится в 4-й колонке
                    rssi = float(parts[3].replace('.', ''))
                    return rssi
    except Exception:
        return None

def calculate_distance(rssi, a, n):
    if rssi is None:
        return 0
    power = (a - rssi) / (10 * n)
    return 10 ** power

# --- НАСТРОЙКИ ---
A = -50   # RSSI на 1 метре. Если у вас вплотную -37, то A можно поднять (например, до -45)
N = 2.5   # Коэффициент затухания (2.0 - открытое пространство, 2.5-4.0 - стены/офис)

# --- НАСТРОЙКИ ФИЛЬТРА ---
WINDOW_SIZE = 10  # Сколько измерений усреднять (чем больше, тем плавнее, но медленнее реакция)

# Инициализация буфера истории (автоматически удаляет старые значения при переполнении)
rssi_history = deque(maxlen=WINDOW_SIZE)

print("--- Wi-Fi Радар (Сглаженный) ---")
print(f"Калибровка: A={A}, N={N}")

try:
    while True:
        raw_rssi = get_rssi_linux()

        if raw_rssi is not None:
            # 1. Добавляем новое значение в историю
            rssi_history.append(raw_rssi)

            # 2. Считаем среднее
            if len(rssi_history) > 0:
                avg_rssi = sum(rssi_history) / len(rssi_history)
            else:
                avg_rssi = raw_rssi

            # 3. Считаем дистанцию по СРЕДНЕМУ значению
            dist = calculate_distance(avg_rssi, A, N)

            print(f"Raw: {raw_rssi} | Smooth: {avg_rssi:.1f} dBm | Dist: {dist:.2f} m")
        else:
            print("Нет сигнала...")

        time.sleep(0.2) # Чуть ускорим опрос (5 раз в секунду)

except KeyboardInterrupt:
    print("\nСтоп.")
