import time
import subprocess
import re
from collections import deque
from picarx import Picarx

# ================== НАСТРОЙКИ ================== #

INTERFACE = "wlan0"     # Wi-Fi интерфейс, подключённый к твоему hotspot'у

# При таком RSSI считаем, что робот "нашёл" тебя.
# По твоим логам около тебя ≈ -55…-60 dBm → начнём с -57.
STOP_RSSI = -40.0

# Сколько раз подряд RSSI должен быть >= STOP_RSSI, чтобы точно остановиться
STOP_HOLD_COUNT = 3

# Период проверки RSSI (сек)
RSSI_CHECK_PERIOD = 0.5

# Окно сглаживания RSSI
RSSI_AVG_WINDOW = 6

# Насколько dB изменение считаем "реальным", а не шумом
DEADBAND_DB = 1.0

# Ультразвук:
# Ниже этого — точно стенка, делаем манёвр
DIST_CRITICAL = 15      # см
# Если ближе этого и сигнал уже хороший — останавливаемся
DIST_SAFE_STOP = 35     # см

# Скорости
SPEED_FORWARD = 26
SPEED_SLOW = 20
SPEED_BACK = 40

# Углы
BIG_TURN_ANGLE = 25        # при "холодно"
SMALL_ZIGZAG_ANGLE = 10    # при шуме

# Калибровка центра руля.
# Если при движении "прямо" его тянет влево — поставь +3..+5.
# Если тянет вправо — -3..-5.
SERVO_TRIM = 0

# =============================================== #

px = Picarx()


class MovingAverage:
    def __init__(self, size=3):
        self.window = deque(maxlen=size)

    def add(self, value):
        self.window.append(value)
        return sum(self.window) / len(self.window) if self.window else value


def get_rssi_connected():
    """
    Возвращает RSSI (dBm) сети, к которой сейчас подключён INTERFACE.
    Если не удалось прочитать — возвращает -100.0.
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

    # 2) Запасной вариант — `iwconfig`
    try:
        out = subprocess.check_output(
            f"iwconfig {INTERFACE}",
            shell=True,
            stderr=subprocess.DEVNULL
        ).decode(errors="ignore")
        m = re.search(r"Signal level= *(-?\d+)\s*dBm", out, re.IGNORECASE)
        if m:
            return max(float(m.group(1)), -100.0)
    except Exception:
        pass

    return -100.0


def read_distance():
    """
    Читаем ультразвук. Всё, что > 250 см или <= 0, считаем "далеко".
    Нам нужны только значения < ~40 см для безопасности.
    """
    d = px.ultrasonic.read()
    if d <= 0 or d > 250:
        return 250.0
    return float(d)


def maneuver_avoid(turn_dir):
    """
    Избегание стены: отъехать назад и повернуть.
    turn_dir: 1 = вправо, -1 = влево (чередуем снаружи).
    """
    print("🛑 СТЕНА! Откат и поворот...")
    px.stop()

    # Назад
    px.backward(SPEED_BACK)
    time.sleep(0.5)

    # Поворот при движении назад
    angle = BIG_TURN_ANGLE * turn_dir
    px.set_dir_servo_angle(SERVO_TRIM + angle)
    px.backward(SPEED_BACK)
    time.sleep(0.4)

    px.set_dir_servo_angle(SERVO_TRIM)
    px.stop()
    time.sleep(0.2)


def main():
    print("\n--- WiFi Find-Me Bot (через hotspot) ---")
    print(f"Интерфейс: {INTERFACE}")
    print(f"Цель: RSSI ≥ {STOP_RSSI:.1f} dBm (держим {STOP_HOLD_COUNT} измерения подряд)\n")

    # Камеру выровняем по центру (если она подключена)
    try:
        px.set_cam_pan_angle(0)
        px.set_cam_tilt_angle(0)
    except Exception:
        pass

    px.set_dir_servo_angle(SERVO_TRIM)
    px.stop()

    avg_filter = MovingAverage(size=RSSI_AVG_WINDOW)

    # --- начальная калибровка RSSI --- #
    print("Калибрую начальный RSSI...")
    base_rssi = get_rssi_connected()
    for _ in range(RSSI_AVG_WINDOW):
        avg_filter.add(base_rssi)
        time.sleep(0.2)

    last_rssi = avg_filter.add(base_rssi)
    print(f"Стартовый RSSI ≈ {last_rssi:.1f} dBm\n")

    last_time = time.time()

    # Направление сильного поворота при "холодно"
    turn_direction = 1    # 1 = вправо, -1 = влево
    # Направление мягкого зигзага при шуме
    zigzag_dir = 1

    # Счётчик, сколько раз подряд RSSI был ≥ STOP_RSSI
    good_rssi_count = 0

    try:
        while True:
            # ---------- 1. Проверка расстояния ---------- #
            dist = read_distance()

            # Очень близко к стене → манёвр
            if dist < DIST_CRITICAL:
                maneuver_avoid(turn_direction)
                turn_direction *= -1  # после манёвра меняем сторону
                last_rssi = avg_filter.add(get_rssi_connected())
                good_rssi_count = 0
                continue

            # Если мы уже близко (< DIST_SAFE_STOP) и сигнал хороший —
            # считаем, что робот рядом с тобой → стоп
            if dist < DIST_SAFE_STOP and last_rssi >= STOP_RSSI - 2.0:
                print(f"🏁 Достаточно близко: dist={dist:.1f} см, RSSI≈{last_rssi:.1f} dBm")
                px.stop()
                break

            # ---------- 2. Обновление RSSI по таймеру ---------- #
            now = time.time()
            if now - last_time >= RSSI_CHECK_PERIOD:
                raw_rssi = get_rssi_connected()
                curr_rssi = avg_filter.add(raw_rssi)
                diff = curr_rssi - last_rssi

                steering = SERVO_TRIM
                speed = SPEED_FORWARD
                status = ""

                # --- учёт целевого RSSI --- #
                if curr_rssi >= STOP_RSSI:
                    good_rssi_count += 1
                else:
                    good_rssi_count = max(0, good_rssi_count - 1)

                if good_rssi_count >= STOP_HOLD_COUNT:
                    print(
                        f"🏆 ПРИБЫЛ! RSSI={curr_rssi:.1f} dBm "
                        f"(≥ {STOP_RSSI:.1f} dBm {good_rssi_count} раз подряд)"
                    )
                    px.stop()
                    break

                # Если сигнал совсем плохой — просто зигзаг и медленно вперёд
                if curr_rssi <= -95.0:
                    status = "📵 Сигнал почти пропал — медленный зигзаг"
                    zigzag_dir *= -1
                    steering = SERVO_TRIM + SMALL_ZIGZAG_ANGLE * zigzag_dir
                    speed = SPEED_SLOW

                else:
                    # ---------- ГОРЯЧО / ХОЛОДНО ---------- #
                    if diff > DEADBAND_DB:
                        # Стало заметно лучше → прямо
                        status = "🟢 Теплее — прямо"
                        steering = SERVO_TRIM
                        speed = SPEED_FORWARD

                    elif diff < -DEADBAND_DB:
                        # Стало заметно хуже → меняем сторону крупного поворота
                        turn_direction *= -1
                        steering = SERVO_TRIM + BIG_TURN_ANGLE * turn_direction
                        speed = SPEED_SLOW
                        dir_icon = "➡️" if turn_direction > 0 else "⬅️"
                        status = f"🔴 Холоднее {dir_icon}"

                    else:
                        # В пределах шума → мягкий зигзаг, чтобы не стоять
                        zigzag_dir *= -1
                        steering = SERVO_TRIM + SMALL_ZIGZAG_ANGLE * zigzag_dir
                        speed = SPEED_FORWARD
                        status = "🟡 Шум — мягкий зигзаг"

                print(
                    f"RSSI: {curr_rssi:.1f} dBm | Δ: {diff:.1f} | "
                    f"dist: {dist:.1f} см | good={good_rssi_count} | {status}"
                )

                px.set_dir_servo_angle(steering)
                px.forward(speed)

                last_rssi = curr_rssi
                last_time = now

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nСтоп по Ctrl+C")
    except Exception as e:
        print("\nОшибка во время работы:", e)
    finally:
        px.stop()
        px.set_dir_servo_angle(SERVO_TRIM)
        print("Робот остановлен.")


if __name__ == "__main__":
    main()
