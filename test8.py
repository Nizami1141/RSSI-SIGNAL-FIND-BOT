import time
from collections import deque
from picarx import Picarx

# --- ИНИЦИАЛИЗАЦИЯ ---
px = Picarx()

# --- НАСТРОЙКИ КАЛИБРОВКИ ---
A = -41    # RSSI на 1 метре (Ваша калибровка)
N = 2.5    # Коэффициент среды

# --- ЦЕЛИ ---
FINISH_THRESHOLD = -46     # Цель (-46 dBm или лучше)
RSSI_TOLERANCE = 1.0       # Игнорируем изменения меньше 1 dBm (чтобы не дергался)

# --- ДВИЖЕНИЕ ---
SPEED_FAST = 40            # Скорость на прямых (оба мотора)
SPEED_TURN = 30            # Скорость на поворотах (оба мотора)
MAX_TURN_ANGLE = 20        # Максимальный угол (было 35, ставим 20 чтобы не крутился)

# --- ПАМЯТЬ РОБОТА ---
WINDOW_SIZE = 10           # Окно сглаживания
rssi_history = deque(maxlen=WINDOW_SIZE)
last_good_rssi = -100      # Запоминаем лучший сигнал
current_angle = 0          # Текущий угол колес

# ================= ФУНКЦИИ =================

def get_rssi_linux():
    try:
        with open("/proc/net/wireless", "r") as f:
            lines = f.readlines()
            for line in lines:
                if "wlan0" in line:
                    parts = line.split()
                    return float(parts[3].replace('.', ''))
    except Exception:
        return None

def get_dist(rssi):
    return 10 ** ((A - rssi) / (10 * N))

def drive(speed, angle):
    """
    Универсальная функция езды.
    speed: скорость обоих моторов.
    angle: угол передних колес (серво).
    """
    # Ограничиваем угол, чтобы не сломать серво и не крутиться
    if angle > MAX_TURN_ANGLE: angle = MAX_TURN_ANGLE
    if angle < -MAX_TURN_ANGLE: angle = -MAX_TURN_ANGLE
    
    px.set_dir_servo_angle(angle)
    px.forward(speed)

def stop_all():
    px.stop()
    px.set_dir_servo_angle(0)

def avoid_obstacle():
    print("⛔ СТЕНА! Отъезжаем...")
    px.stop()
    time.sleep(0.2)
    px.set_dir_servo_angle(0) # Колеса прямо
    px.backward(40)           # Оба мотора назад
    time.sleep(1.0)
    rssi_history.clear()      # Сброс памяти Wi-Fi

# ================= ГЛАВНЫЙ ЦИКЛ =================

print(f"--- Anti-Spin Wi-Fi Bot (A={A}) ---")
stop_all()
time.sleep(2)

try:
    # Направление для маневра, если сигнал падает (1 = вправо, -1 = влево)
    seek_dir = 1 

    while True:
        # 1. ПРОВЕРКА ПРЕПЯТСТВИЙ
        dist_cm = px.ultrasonic.read()
        if 0 < dist_cm < 20:
            avoid_obstacle()
            continue

        # 2. ПОЛУЧЕНИЕ WI-FI
        raw = get_rssi_linux()
        if raw is None: continue
        
        rssi_history.append(raw)
        if len(rssi_history) < WINDOW_SIZE:
            time.sleep(0.1)
            continue

        # Сглаженный сигнал
        avg_rssi = sum(rssi_history) / len(rssi_history)
        est_dist = get_dist(avg_rssi)

        # 3. ЛОГИКА "НЕ КРУТИСЬ"
        
        # Если мы достигли цели
        if avg_rssi >= FINISH_THRESHOLD:
            print(f"🎉 ФИНИШ! ({avg_rssi:.1f} dBm)")
            stop_all()
            break

        # Сравниваем с предыдущим "хорошим" замером
        delta = avg_rssi - last_good_rssi

        # --- СЦЕНАРИЙ А: СИГНАЛ РАСТЕТ ИЛИ СТАБИЛЕН ---
        # Мы идем верно. Газуем обоими моторами ПРЯМО.
        if delta >= -RSSI_TOLERANCE: 
            # Даже если чуть упал (в пределах шума), считаем что все ок.
            # Выравниваем колеса в 0
            current_angle = 0 
            last_good_rssi = avg_rssi # Обновляем эталон
            
            print(f"✅ ОК ({avg_rssi:.1f}) -> ПРЯМО")
            drive(SPEED_FAST, 0)

        # --- СЦЕНАРИЙ Б: СИГНАЛ ЯВНО УХУДШИЛСЯ ---
        # Мы уезжаем от роутера. Нужно плавно подрулить.
        else:
            print(f"❄️ ХУЖЕ ({avg_rssi:.1f}) -> ", end="")
            
            # Если мы ехали прямо и сигнал упал -> начинаем поворот
            if current_angle == 0:
                current_angle = MAX_TURN_ANGLE * seek_dir
                print(f"НАЧИНАЮ ПОВОРОТ {seek_dir}")
            
            # Если мы УЖЕ поворачивали, и сигнал ВСЕ РАВНО падает -> 
            # Значит поворачиваем не туда! Меняем сторону.
            else:
                seek_dir *= -1 # Меняем 1 на -1 (или наоборот)
                current_angle = MAX_TURN_ANGLE * seek_dir
                print(f"МЕНЯЮ СТОРОНУ -> {seek_dir}")
                # Сбрасываем "эталон", чтобы дать шанс новому направлению
                last_good_rssi = avg_rssi 

            drive(SPEED_TURN, current_angle)

        time.sleep(0.1) # Частые проверки

except KeyboardInterrupt:
    stop_all()
    print("\nСтоп.")
