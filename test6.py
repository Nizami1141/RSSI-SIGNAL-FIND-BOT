#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
PicarX Wi-Fi Find-Me (умный поиск по RSSI + ультразвук)

Фичи:
- Wi-Fi "радар" с усреднением (для калибровки RSSI/дистанции)
- Три фазы движения:
    1) 🚀 Быстрый подход   (слабый сигнал, далеко)
    2) 🎯 Точная наводка   (середина, замедляемся)
    3) 🏁 Финиш            (очень сильный сигнал, много раз подряд)
- Объезд препятствий по ультразвуку
- Логи в духе того, что ты кидал (но «ПРИБЫЛИ!» теперь строже)
"""

import time
import math
import subprocess
from collections import deque

try:
    from picarx import Picarx
except ImportError:
    # На всякий случай, чтобы файл хотя бы импортировался без робота
    class Picarx:
        def forward(self, speed): pass
        def backward(self, speed): pass
        def stop(self): pass
        def set_dir_servo_angle(self, angle): pass
        def get_distance(self): return 100.0


# ==========================
#   Wi-Fi Distance Tracker
# ==========================

class WiFiDistanceTracker:
    """
    Получает RSSI текущей Wi-Fi связи и оценивает расстояние по модели
    log-distance path loss.

    dist(d) = 10 ^ ((A - RSSI) / (10 * n))
        A — RSSI на 1 м (дБм)
        n — коэффициент затухания среды
    """

    def __init__(
        self,
        interface="wlan0",
        A=-50.0,
        n=2.5,
        window_size=10,
    ):
        self.interface = interface
        self.A = A
        self.n = n
        self.window_size = window_size
        self.rssi_window = deque(maxlen=window_size)

    # ---------- низкоуровневый RSSI ----------

    def _get_rssi_raw(self):
        """
        Достаём RSSI из вывода `iw dev wlan0 link` или `iwconfig`.
        Возвращаем float (дБм) или None, если не получилось.
        """
        # Пробуем `iw dev ... link`
        try:
            out = subprocess.check_output(
                ["iw", "dev", self.interface, "link"],
                stderr=subprocess.DEVNULL,
                universal_newlines=True,
            )
            for line in out.splitlines():
                line = line.strip()
                # Примеры:
                #   signal: -58 dBm
                if line.startswith("signal:"):
                    parts = line.split()
                    for p in parts:
                        try:
                            return float(p)
                        except ValueError:
                            continue
        except Exception:
            pass

        # Если не сработало — пробуем iwconfig
        try:
            out = subprocess.check_output(
                ["iwconfig", self.interface],
                stderr=subprocess.DEVNULL,
                universal_newlines=True,
            )
            for line in out.splitlines():
                line = line.strip()
                # Примеры:
                #   Link Quality=70/70  Signal level=-40 dBm
                if "Signal level" in line:
                    # ищем "-XX" перед "dBm"
                    segments = line.replace("=", " ").replace("dBm", " ").split()
                    for seg in segments:
                        if seg.startswith("-"):
                            try:
                                return float(seg)
                            except ValueError:
                                continue
        except Exception:
            pass

        return None

    # ---------- сглаживание и расстояние ----------

    def get_rssi_smooth(self):
        """
        Возвращает (raw, smooth), где:
          raw    — сырое значение RSSI
          smooth — скользящее среднее по окну
        Если raw не удалось получить — оба значения None.
        """
        raw = self._get_rssi_raw()
        if raw is None:
            return None, None

        self.rssi_window.append(raw)
        smooth = sum(self.rssi_window) / len(self.rssi_window)
        return raw, smooth

    def rssi_to_distance(self, rssi):
        """
        Оценка расстояния по модели затухания.
        """
        if rssi is None:
            return None
        # d = 10 ^ ((A - RSSI) / (10 * n))
        exponent = (self.A - rssi) / (10.0 * self.n)
        return 10.0 ** exponent

    def get_distance(self):
        """
        Комплексный вызов: даёт (raw_rssi, smooth_rssi, distance_m).
        """
        raw, smooth = self.get_rssi_smooth()
        if smooth is None:
            return raw, None, None
        dist = self.rssi_to_distance(smooth)
        return raw, smooth, dist


# ==========================
#       Движение PicarX
# ==========================

# Скорости
FAST_SPEED = 50     # быстрый подход
PRECISE_SPEED = 20  # точная подводка

# Ультразвук
OBSTACLE_DISTANCE_CM = 20.0  # всё ближе — считаем препятствием

# Поиск по поворотам
SEARCH_ANGLE = 35           # макс. угол при "ИЩУ"
SEARCH_STEP_ANGLE = 15      # шаг туда-сюда
SEARCH_PAUSE = 0.20         # пауза между шагами

# Фазы по RSSI (дБм)
FAST_PHASE_LIMIT = -50.0    # ниже — далеко, едем быстро
PRECISE_PHASE_LIMIT = -46.0 # между — точная наводка
FINISH_RSSI_THRESHOLD = -44.0  # условный "очень сильный" сигнал
FINISH_STRONG_COUNT = 12       # сколько раз подряд надо держать сильный сигнал

# Стабильность сигнала (чтобы не орать ПРИБЫЛИ! на каждом пике)
STABLE_WINDOW = 8      # сколько последних измерений смотрим
STABLE_SPREAD = 3.0    # dB — диапазон, в котором считается, что сигнал стабилен


def ultrasonic_distance_cm(px: Picarx) -> float:
    """
    Обёртка для get_distance() с безопасным fallback.
    """
    try:
        d = px.get_distance()
        if d is None:
            return 999.0
        return float(d)
    except Exception:
        return 999.0


def avoid_obstacle(px: Picarx, last_turn_side: int):
    """
    Простейший объезд препятствия:
    - стоп
    - немного сдаём назад
    - поворачиваем в сторону, противоположную last_turn_side
    """
    dist = ultrasonic_distance_cm(px)
    print(f"⛔ ПРЕПЯТСТВИЕ ({dist:.2f}см) -> ОТЪЕЗД")

    px.stop()
    time.sleep(0.1)

    # немного назад
    px.backward(FAST_SPEED)
    time.sleep(0.4)
    px.stop()

    # поворот
    if last_turn_side >= 0:
        # последний поворот был вправо или ещё не было — теперь влево
        angle = -SEARCH_ANGLE
    else:
        angle = SEARCH_ANGLE

    px.set_dir_servo_angle(angle)
    time.sleep(0.3)
    px.set_dir_servo_angle(0)


def smart_search(px: Picarx, tracker: WiFiDistanceTracker):
    """
    Главный цикл поиска:
      - Следим за RSSI
      - Едем вперёд, если нет препятствий
      - Если RSSI не улучшается — делаем поисковый поворот
      - Если сигнал очень сильный и стабильный — считаем, что приехали
    """

    print("\n--- PicarX: Smart Search ---")
    print("1. Быстрый подход (< -50 dBm)")
    print("2. Точная наводка (-50 ... -46 dBm)")
    print("3. Финиш (>= -44 dBm, долго и стабильно)\n")

    last_turn_side = 1  # 1 = вправо, -1 = влево
    search_direction = 1
    strong_count = 0    # сколько подряд сильных RSSI
    stable_rssi_window = deque(maxlen=STABLE_WINDOW)

    try:
        while True:
            # --- Получаем Wi-Fi RSSI и дистанцию ---
            raw, smooth, dist = tracker.get_distance()

            if smooth is None:
                print("⚠️  Нет данных RSSI, жду...")
                px.stop()
                time.sleep(0.5)
                continue

            # --- Ультразвук ---
            u_dist = ultrasonic_distance_cm(px)

            # --- Определяем фазу по RSSI ---
            if smooth < FAST_PHASE_LIMIT:
                phase = "FAST"
                phase_label = "🚀 БЫСТРО"
                speed = FAST_SPEED
            elif smooth < PRECISE_PHASE_LIMIT:
                phase = "PRECISE"
                phase_label = "🎯 ТОЧНО"
                speed = PRECISE_SPEED
            else:
                phase = "FINISH"
                phase_label = "🏁 ФИНИШ"
                speed = PRECISE_SPEED

            # --- Обновляем окна стабильности ---
            stable_rssi_window.append(smooth)

            # --- Проверяем условие финиша ---
            if smooth >= FINISH_RSSI_THRESHOLD:
                strong_count += 1
            else:
                strong_count = 0

            finish_reached = False
            if strong_count >= FINISH_STRONG_COUNT and len(stable_rssi_window) == STABLE_WINDOW:
                # Проверка: диапазон RSSI мал -> стабильно
                spread = max(stable_rssi_window) - min(stable_rssi_window)
                if spread <= STABLE_SPREAD:
                    finish_reached = True

            # Лог состояния
            if dist is not None:
                dist_str = f"{dist:.2f} м"
            else:
                dist_str = "N/A"

            # Печатаем строку состояния
            # Пример: [🚀 БЫСТРО] -59.3 dBm | ✅ ПРЯМО (Spd: 50)
            state_text = ""
            action_text = ""

            # --- Логика движения ---

            if finish_reached:
                px.stop()
                print()
                print("========================================")
                print(f"🏁 ПРИБЫЛИ! Сигнал: {smooth:.1f} dBm, dist≈{dist_str}")
                print("========================================")
                break

            # Препятствие вплотную — объезд всегда в приоритете
            if u_dist < OBSTACLE_DISTANCE_CM:
                state_text = f"[{phase_label}] {smooth:.1f} dBm (dist≈{dist_str})"
                action_text = "⛔ ПРЕПЯТСТВИЕ -> ОТЪЕЗД"
                print(f"{state_text} | {action_text}")
                avoid_obstacle(px, last_turn_side)
                # после объезда считаем, что последний поворот был туда же
                last_turn_side = -last_turn_side
                time.sleep(0.1)
                continue

            # Фаза FINISH: почти не крутим руль, аккуратно подкрадываемся
            if phase == "FINISH":
                px.set_dir_servo_angle(0)
                px.forward(speed)
                state_text = f"[{phase_label}] {smooth:.1f} dBm (dist≈{dist_str})"
                action_text = f"✅ ПРЯМО (Spd: {speed})"
                print(f"{state_text} | {action_text}")
                time.sleep(0.25)
                continue

            # Для FAST и PRECISE: чередуем "еду прямо" и "ищу"
            # Простейшая эвристика: если последняя порция RSSI стала хуже — крутим руль
            if len(stable_rssi_window) >= 2:
                prev = list(stable_rssi_window)[-2]
            else:
                prev = smooth

            if smooth >= prev - 0.3:
                # Сигнал не хуже — едем прямо
                px.set_dir_servo_angle(0)
                px.forward(speed)
                state_text = f"[{phase_label}] {smooth:.1f} dBm (dist≈{dist_str})"
                action_text = f"✅ ПРЯМО (Spd: {speed})"
                print(f"{state_text} | {action_text}")
                time.sleep(0.25)
            else:
                # Сигнал заметно просел — пробуем поиск поворотом
                search_angle = search_direction * SEARCH_ANGLE
                px.set_dir_servo_angle(search_angle)
                px.forward(speed)
                state_text = f"[{phase_label}] {smooth:.1f} dBm (dist≈{dist_str})"
                action_text = f"❄️ ИЩУ (Ang: {abs(search_angle)})"
                print(f"{state_text} | {action_text}")
                time.sleep(SEARCH_PAUSE)

                # Меняем направление для следующего поиска
                search_direction *= -1
                last_turn_side = 1 if search_angle > 0 else -1

    except KeyboardInterrupt:
        print("\nСтоп по Ctrl+C.")
    finally:
        px.stop()
        px.set_dir_servo_angle(0)
        time.sleep(0.2)


def wifi_radar_calibration(tracker: WiFiDistanceTracker, duration_sec=10):
    """
    Небольшой радар, как в твоих логах:
    печатаем Raw / Smooth / Dist.
    """
    print("--- Wi-Fi Радар (Сглаженный) ---")
    print(f"Калибровка: A={tracker.A}, N={tracker.n}, окно={tracker.window_size}")
    start = time.time()

    try:
        while time.time() - start < duration_sec:
            raw, smooth, dist = tracker.get_distance()
            if raw is None:
                print("Raw: N/A | Smooth: N/A | Dist: N/A")
            else:
                if dist is None:
                    dist_str = "N/A"
                else:
                    dist_str = f"{dist:.2f} м"
                print(f"Raw: {raw:.1f} | Smooth: {smooth:.1f} dBm | Dist: {dist_str}")
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("^C")
    finally:
        print("Стоп.")


def main():
    px = Picarx()

    # Инициализируем трекер Wi-Fi
    tracker = WiFiDistanceTracker(
        interface="wlan0",
        A=-50.0,
        n=2.5,
        window_size=10,
    )

    # 1) Небольшая калибровка (радар). Можно уменьшить/увеличить время по вкусу.
    wifi_radar_calibration(tracker, duration_sec=10)

    # 2) Запускаем умный поиск
    smart_search(px, tracker)


if __name__ == "__main__":
    main()
