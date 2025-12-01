import time
import subprocess
import re
from collections import deque
from picarx import Picarx

# ------------ НАСТРОЙКИ ------------ #

INTERFACE = "wlan0"

# Вместо жёсткого TARGET_RSSI используем относительное улучшение от старта
TARGET_DELTA_DB = 8.0      # Насколько dB сигнал должен стать лучше стартового

# Если очень хочется, можно ещё задать "минимум"
MIN_ABS_RSSI = -35.0       # Не останавливаемся, если сигнал всё ещё хуже этого

# Сколько раз подряд должно выполняться условие, чтобы реально остановиться
TARGET_HOLD_COUNT = 4

WIFI_CHECK_DELAY = 0.3

# Ультразвук
DIST_CRITICAL = 18         # Меньше этого — точно стена → манёвр
STOP_DIST = 40             # Если ближе этого и сигнал хороший — считаем, что нашли

# Логика поворота
KP = 4.0
DEADBAND = 1.5             # мёртвая зона по ΔRSSI
MAX_STEER_ANGLE = 35

SPEED_FORWARD = 35
SPEED_SEARCH = 25
SPEED_BACKWARD = 45

RSSI_AVG_WINDOW = 4

px = Picarx()

# ------------ ВСПОМОГАТЕЛЬНЫЕ ШТУКИ ------------ #

class MovingAverage:
    def __init__(self, size=3):
        self.window = deque(maxlen=size)
    def add(self, value):
        self.window.append(value)
        return sum(self.window) / len(self.window) if self.window else value

def get_rssi_connected():
    """
    RSSI (dBm) сети, к которой сейчас подключен INTERFACE.
    """
    # 1) Через `iw dev ... link`
    try:
        out = subprocess.check_output(
            f"iw dev {INTERFACE} link",
            shell=True,
            stderr=subprocess.DEVNULL
        ).decode(errors="ignore")
        m = re.search(r"signal:\s*(-?\d+\.?\d*)\s*dBm", out)
        if m:
            rssi = float(m.group(1))
            return max(rssi, -100.0)
    except Exception:
        pass

    # 2) Через `iwconfig`
    try:
        out = subprocess.check_output(
            f"iwconfig {INTERFACE}",
            shell=True,
            stderr=subprocess.DEVNULL
        ).decode(errors="ignore")
        m = re.search(r"Signal level= *(-?\d+)\s*dBm", out, re.IGNORECASE)
        if m:
            rssi = float(m.group(1))
            return max(rssi, -100.0)
    except Exception:
        pass

    return -100.0

def maneuver_avoid():
    """Грубое избегание стены: откат назад и поворот."""
    print("🛑 СТЕНА! Агрессивный откат...")
    px.stop()

    px.backward(SPEED_BACKWARD)
    time.sleep(0.6)

    px.set_dir_servo_angle(-30)
    px.backward(int(SPEED_BACKWARD * 0.7))
    time.sleep(0.5)

    px.set_dir_servo_angle(0)
    px.stop()
    time.sleep(0.2)

# ------------ ОСНОВНОЙ ЦИКЛ ------------ #

def main():
    print("\n--- Find Me Bot (динамический порог по RSSI) ---")
    print(f"Интерфейс: {INTERFACE}")
    print(f"Цель: улучшить RSSI минимум на {TARGET_DELTA_DB} dB от стартового,\n"
          f"      но не хуже {MIN_ABS_RSSI} dBm и не ближе {STOP_DIST} см по ультразвуку.\n")

    # Камера по центру (если есть)
    try:
        px.set_cam_pan_angle(0)
        px.set_cam_tilt_angle(0)
    except Exception:
        pass

    px.set_dir_servo_angle(0)
    px.stop()

    avg_filter = MovingAverage(size=RSSI_AVG_WINDOW)

    # Стартовая калибровка
    print("Калибрую стартовый RSSI...")
    base_rssi = get_rssi_connected()
    for _ in range(RSSI_AVG_WINDOW):
        avg_filter.add(base_rssi)
        time.sleep(0.2)

    start_rssi = avg_filter.add(base_rssi)
    target_rssi = max(start_rssi + TARGET_DELTA_DB, MIN_ABS_RSSI)

    print(f"Стартовый RSSI ≈ {start_rssi:.1f} dBm")
    print(f"Буду считать 'нашёл', когда RSSI ≥ {target_rssi:.1f} dBm "
          f"и это подтвердится {TARGET_HOLD_COUNT} раз подряд.\n")

    last_rssi = start_rssi
    last_time = time.time()
    turn_direction = 1       # 1 = вправо, -1 = влево

    in_target_count = 0

    try:
        while True:
            # 1. Стена / препятствие
            dist = px.ultrasonic.read()
            if dist <= 0:
                dist = 100

            if dist < DIST_CRITICAL:
                maneuver_avoid()
                turn_direction *= -1
                last_rssi = avg_filter.add(get_rssi_connected())
                in_target_count = 0
                continue

            # 2. Обновляем RSSI
            if time.time() - last_time >= WIFI_CHECK_DELAY:
                raw_rssi = get_rssi_connected()
                curr_rssi = avg_filter.add(raw_rssi)
                diff = curr_rssi - last_rssi

                steering_angle = 0
                speed = SPEED_FORWARD
                status = ""

                # --- Условие "нашёл" по RSSI --- #
                if curr_rssi >= target_rssi:
                    in_target_count += 1
                else:
                    in_target_count = max(in_target_count - 1, 0)

                # --- Доп. стоп по расстоянию, чтобы не приезжать в упор --- #
                if dist < STOP_DIST and curr_rssi > (start_rssi + 3.0):
                    print(
                        f"🏁 Достаточно близко: dist={dist:.1f} см, "
                        f"RSSI={curr_rssi:.1f} dBm (> старт + 3 dB)"
                    )
                    px.stop()
                    break

                # Если условие по RSSI подтвердилось несколько раз — тоже стоп
                if in_target_count >= TARGET_HOLD_COUNT:
                    print(
                        f"🏆 ПРИБЫЛ! RSSI={curr_rssi:.1f} dBm "
                        f"(порог {target_rssi:.1f} dBm, удержано {in_target_count} раз)"
                    )
                    px.stop()
                    break

                # --- Особый случай: сигнал совсем пропал --- #
                if curr_rssi <= -95.0:
                    status = "📵 СИГНАЛ ПРОПАЛ — разворачиваюсь и ищу"
                    # Поменяем сторону и чуть повернёмся
                    turn_direction *= -1
                    steering_angle = MAX_STEER_ANGLE * turn_direction
                    speed = SPEED_SEARCH
                    print(
                        f"RSSI: {curr_rssi:.1f} dBm | Δ: {diff:.1f} | dist: {dist:.1f} см | "
                        f"in_target={in_target_count} | {status}"
                    )
                    px.set_dir_servo_angle(steering_angle)
                    px.forward(speed)
                    last_rssi = curr_rssi
                    last_time = time.time()
                    time.sleep(0.05)
                    continue

                # --- ЛОГИКА «ГОРЯЧО / ХОЛОДНО» --- #

                if diff >= -0.5:
                    # Стало лучше или почти не изменилось → прямо
                    status = "🟢 ГОРЯЧЕЕ — прямо"
                    steering_angle = 0
                    speed = SPEED_FORWARD
                else:
                    # Стало хуже
                    if abs(diff) < DEADBAND:
                        # Считаем шумом → чуть медленнее, но прямо
                        status = "🟡 ШУМ — ровнее, чуть медленнее"
                        steering_angle = 0
                        speed = int(SPEED_FORWARD * 0.9)
                    else:
                        # Реально холоднее → МЕНЯЕМ сторону зигзага
                        error = abs(diff)
                        angle = KP * error
                        if angle > MAX_STEER_ANGLE:
                            angle = MAX_STEER_ANGLE

                        # ВАЖНО: здесь сразу инвертируем направление,
                        # чтобы не тащило всё время в одну сторону
                        turn_direction *= -1

                        steering_angle = angle * turn_direction
                        speed = SPEED_SEARCH
                        dir_icon = "➡️" if turn_direction > 0 else "⬅️"
                        status = f"🔴 ХОЛОДНО {dir_icon} (руль {steering_angle:.1f}°)"

                print(
                    f"RSSI: {curr_rssi:.1f} dBm | Δ: {diff:.1f} | dist: {dist:.1f} см | "
                    f"in_target={in_target_count} | {status}"
                )

                px.set_dir_servo_angle(steering_angle)
                px.forward(speed)

                last_rssi = curr_rssi
                last_time = time.time()

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nСтоп по Ctrl+C")
    except Exception as e:
        print("\nОшибка во время работы:", e)
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        print("Робот остановлен.")

if __name__ == "__main__":
    main()
