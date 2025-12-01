#!/usr/bin/env python3
import time
from collections import deque
from picarx import Picarx

# ========== ТВОЙ Wi-Fi РАДАР (ПЕРЕНОС ИЗ signal_measure.py) ========== #

def get_rssi_linux():
    """
    Читаем RSSI из /proc/net/wireless.
    Возвращаем значение в dBm (float) или None, если не нашли wlan0.
    """
    try:
        with open("/proc/net/wireless", "r") as f:
            lines = f.readlines()
        for line in lines:
            if "wlan0" in line:          # если у тебя другой интерфейс — поменяй!
                parts = line.split()
                # В /proc/net/wireless поле уровня выглядит типа "-73."
                rssi = float(parts[3].replace('.', ''))
                return rssi
        return None
    except Exception:
        return None


def calculate_distance(rssi, a, n):
    """
    Модель затухания:
        d = 10 ^ ((A - RSSI) / (10 * n))

    A — RSSI на 1 метре,
    n — коэффициент среды (2..4).
    """
    if rssi is None:
        return 0.0
    power = (a - rssi) / (10.0 * n)
    return 10 ** power


# --- НАСТРОЙКИ МОДЕЛИ СИГНАЛА --- #

A = -50.0   # твоё значение "RSSI на 1 м". Подстраивай под свои замеры.
N = 2.5     # коэффициент среды (2-3 для indoors)

WINDOW_SIZE = 10   # сколько последних измерений усреднять (как у тебя в радаре)

# ========== НАСТРОЙКИ ДВИЖЕНИЯ РОБОТА ========== #

# Желаемая дистанция до "маяка"
TARGET_DIST_M = 2.0       # хотим примерно 2 метра
DIST_TOL_M = 0.5          # допускаем коридор ±0.5 м → 1.5–2.5 м

# Насколько изменение дистанции считаем реальным (а не шумом)
DIST_IMPROVE_MIN = 0.4    # м – если ближе/дальше больше чем на это, считаем значимым

# Частота обновления Wi-Fi (сек)
WIFI_PERIOD = 0.4

# Ультразвук (см)
DIST_CRITICAL = 15        # меньше — стенка, делаем откат
DIST_STOP_SAFE = 35       # если ближе этого и уже в целевом диапазоне — стоп

# Скорости
SPEED_FORWARD = 26
SPEED_SLOW = 20
SPEED_BACK = 40

# Углы поворота
BIG_TURN_ANGLE = 25       # при "холодно"
SMALL_ZIGZAG_ANGLE = 10   # при шуме

# Калибровка центра руля
# Если его "прямо" чуть тянет влево → поставь +3..+5.
# Если тянет вправо → -3..-5.
SERVO_TRIM = 0

px = Picarx()


# ========== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ========== #

class MovingAverage:
    def __init__(self, size=3):
        self.window = deque(maxlen=size)

    def add(self, value):
        self.window.append(value)
        return sum(self.window) / len(self.window) if self.window else value


def read_distance_ultra():
    """
    Читаем ультразвук. Всё, что > 250 см или <= 0, считаем "очень далеко".
    Нам важно только < ~40 см.
    """
    d = px.ultrasonic.read()
    if d <= 0 or d > 250:
        return 250.0
    return float(d)


def maneuver_avoid(turn_dir):
    """
    Жёсткий откат от стены: назад + поворот.
    turn_dir: 1 = вправо, -1 = влево.
    """
    print("🛑 СТЕНА! Откат и поворот...")
    px.stop()

    px.backward(SPEED_BACK)
    time.sleep(0.5)

    angle = BIG_TURN_ANGLE * turn_dir
    px.set_dir_servo_angle(SERVO_TRIM + angle)
    px.backward(SPEED_BACK)
    time.sleep(0.4)

    px.set_dir_servo_angle(SERVO_TRIM)
    px.stop()
    time.sleep(0.2)


# ========== ОСНОВНОЙ АЛГОРИТМ ========== #

def main():
    print("\n--- PicarX Wi-Fi Distance Find-Me ---")
    print(f"Модель: A={A}, N={N}, окно усреднения={WINDOW_SIZE}")
    print(f"Цель: дистанция ≈ {TARGET_DIST_M} м (коридор ±{DIST_TOL_M} м)\n")

    # Пытаемся выровнять камеру по центру (если есть)
    try:
        px.set_cam_pan_angle(0)
        px.set_cam_tilt_angle(0)
    except Exception:
        pass

    px.set_dir_servo_angle(SERVO_TRIM)
    px.stop()

    avg_filter = MovingAverage(size=WINDOW_SIZE)

    # --- калибруем начальное расстояние по твоей модели --- #
    print("Калибрую начальное расстояние по Wi-Fi...")
    rssi0 = get_rssi_linux()
    if rssi0 is None:
        print("❌ Не вижу wlan0 в /proc/net/wireless. Выход.")
        return

    for _ in range(WINDOW_SIZE):
        r = get_rssi_linux()
        if r is not None:
            avg_filter.add(r)
        time.sleep(0.2)

    avg_rssi = avg_filter.add(get_rssi_linux())
    dist = calculate_distance(avg_rssi, A, N)
    last_dist = dist

    print(f"Старт: RSSI≈{avg_rssi:.1f} dBm, Dist≈{dist:.2f} м\n")

    last_wifi_time = time.time()

    turn_direction = 1   # для больших поворотов (1=вправо, -1=влево)
    zigzag_dir = 1       # для мягкого зигзага

    try:
        while True:
            # -------- 1. Стенки по ультразвуку -------- #
            d_ultra = read_distance_ultra()

            if d_ultra < DIST_CRITICAL:
                maneuver_avoid(turn_direction)
                turn_direction *= -1
                # Обновляем оценку дистанции после манёвра
                r = get_rssi_linux()
                if r is not None:
                    avg_rssi = avg_filter.add(r)
                    dist = calculate_distance(avg_rssi, A, N)
                    last_dist = dist
                continue

            # -------- 2. Обновление Wi-Fi-дистанции -------- #
            now = time.time()
            if now - last_wifi_time >= WIFI_PERIOD:
                raw_rssi = get_rssi_linux()
                if raw_rssi is None:
                    print("📵 Нет сигнала от wlan0, останавливаюсь.")
                    px.stop()
                    break

                avg_rssi = avg_filter.add(raw_rssi)
                dist = calculate_distance(avg_rssi, A, N)
                delta_d = last_dist - dist   # >0 — стали БЛИЖЕ

                # --- проверка, что мы уже в нужном коридоре --- #
                in_window = abs(dist - TARGET_DIST_M) <= DIST_TOL_M

                if in_window and d_ultra < DIST_STOP_SAFE:
                    print(
                        f"🏆 ПРИБЫЛ! Dist≈{dist:.2f} м "
                        f"(целевой {TARGET_DIST_M}±{DIST_TOL_M}), "
                        f"ультразвук {d_ultra:.1f} см"
                    )
                    px.stop()
                    break

                steering = SERVO_TRIM
                speed = SPEED_FORWARD
                status = ""

                # ---------- ЛОГИКА ПО ДИСТАНЦИИ (ГОРЯЧО/ХОЛОДНО) ---------- #

                if delta_d > DIST_IMPROVE_MIN:
                    # Стали ощутимо БЛИЖЕ → едем прямо
                    status = "🟢 БЛИЖЕ — прямо"
                    steering = SERVO_TRIM
                    speed = SPEED_FORWARD

                elif delta_d < -DIST_IMPROVE_MIN:
                    # Стали ДАЛЬШЕ → меняем направление поворота
                    turn_direction *= -1
                    steering = SERVO_TRIM + BIG_TURN_ANGLE * turn_direction
                    speed = SPEED_SLOW
                    dir_icon = "➡️" if turn_direction > 0 else "⬅️"
                    status = f"🔴 ДАЛЬШЕ {dir_icon} (поворот)"

                else:
                    # Δ маленькое → похоже на шум; мягкий зигзаг
                    zigzag_dir *= -1
                    steering = SERVO_TRIM + SMALL_ZIGZAG_ANGLE * zigzag_dir
                    speed = SPEED_FORWARD
                    status = "🟡 ШУМ — мягкий зигзаг"

                print(
                    f"Raw: {raw_rssi:.1f} | Smooth: {avg_rssi:.1f} dBm | "
                    f"Dist: {dist:.2f} м | Δd={delta_d:+.2f} м | "
                    f"Ultrasonic: {d_ultra:.1f} см | {status}"
                )

                px.set_dir_servo_angle(steering)
                px.forward(speed)

                last_dist = dist
                last_wifi_time = now

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
