#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Based on your test12 (1).py:
- Keeps ultrasonic as HARD safety (immediate AVOID)
- Adds "smart camera" vision assist:
    * Estimates free space Left/Right/Ahead from the camera image (no heavy AI model required)
    * Uses vision to steer away from clutter while driving
    * Uses vision to pick the better turn direction during AVOID (with ultrasonic scan as fallback)

If OpenCV/camera is not available, the robot runs exactly like your original ultrasonic-only logic.

Requirements (optional):
  pip install opencv-python numpy
"""

import time
from typing import Optional, Dict, Any, Tuple

from picarx import Picarx

# ================= КОНФИГУРАЦИЯ (как у тебя) =================
TARGET_RSSI = -46         # Цель (dBm) - уровень сигнала рядом с роутером
APPROACH_THRESHOLD = -50  # Порог начала сближения
PROCESS_NOISE_Q = 0.05    # Шум процесса (фильтр Калмана)
MEASUREMENT_NOISE_R = 2.0 # Шум измерений (чем больше, тем плавнее график)
MAX_STEER_ANGLE = 35      # Максимальный угол поворота (защита механики)
OBSTACLE_DIST_CM = 25     # Дистанция до препятствия (см)
SPEED_SEARCH = 40         # Скорость в режиме поиска
SPEED_APPROACH = 30       # Скорость в режиме сближения

# ================= SMART CAMERA (Vision) =================
VISION_ENABLE = True               # Включить "умную камеру" (если камера + OpenCV доступны)
VISION_CAM_INDEX = 0              # Индекс камеры для OpenCV VideoCapture(0)
VISION_WIDTH = 320
VISION_HEIGHT = 240
VISION_FPS_LIMIT = 8.0            # Ограничение частоты обработки (экономит CPU на Raspberry Pi 4)
VISION_ROI_BOTTOM_RATIO = 0.55    # Берём нижнюю часть кадра (ближайшая зона перед роботом)
VISION_CENTER_WIDTH_RATIO = 0.45  # Ширина центральной зоны "впереди"
VISION_FREE_AHEAD_MIN = 0.55      # Если свободно впереди меньше → считаем опасным → AVOID
VISION_STEER_GAIN = 1.35          # Усиление руления по vision
VISION_DILATE_ITERS = 2           # Утолщение контуров (устойчивее к шуму)
VISION_DEBUG = False              # Показ окна (НЕ рекомендуется на headless)

# ================= Вспомогательные =================
def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


# ================= КЛАССЫ =================

class RSSIKalmanFilter:
    """Простой 1D фильтр Калмана для сглаживания RSSI."""
    def __init__(self, R, Q, initial_value=-70):
        self.R = R
        self.Q = Q
        self.x = initial_value
        self.P = 1.0

    def filter(self, measurement):
        if measurement is None:
            return self.x
        p_pred = self.P + self.Q
        K = p_pred / (p_pred + self.R)
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * p_pred
        return self.x


class WiFiSensor:
    """Чтение уровня RSSI из /proc/net/wireless."""
    def __init__(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def get_rssi(self):
        try:
            with open(self.filepath, "r") as f:
                for line in f:
                    if self.interface + ":" in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            rssi_raw = parts[3]
                            try:
                                return float(rssi_raw)
                            except ValueError:
                                return float(rssi_raw.replace(".", ""))
        except Exception as e:
            print(f"Ошибка чтения RSSI: {e}")
            return None
        return None


class SmartCameraAvoider:
    """
    Lightweight "smart camera" module:
    - Reads frames (PiCamera2 if available, else OpenCV VideoCapture)
    - Computes obstacle density using edges in bottom ROI
    - Produces free_left / free_right / free_ahead scores in [0..1]
    """
    def __init__(
        self,
        enabled: bool = True,
        cam_index: int = 0,
        width: int = 320,
        height: int = 240,
        fps_limit: float = 8.0,
        debug: bool = False,
    ):
        self.enabled_requested = bool(enabled)
        self.enabled = False
        self.debug = bool(debug)
        self.fps_limit = float(fps_limit)
        self.width = int(width)
        self.height = int(height)

        self._last_t = 0.0
        self._last_scores: Optional[Dict[str, Any]] = None

        # Optional OpenCV
        try:
            import cv2  # type: ignore
            import numpy as np  # type: ignore
            self.cv2 = cv2
            self.np = np
        except Exception:
            self.cv2 = None
            self.np = None

        self._mode = "none"
        self._cap = None
        self._picam2 = None

        if not self.enabled_requested:
            return

        if self.cv2 is None:
            print("[Vision] OpenCV не найден. Продолжаю без vision.")
            return

        # Try PiCamera2 first
        try:
            from picamera2 import Picamera2  # type: ignore
            self._picam2 = Picamera2()
            cfg = self._picam2.create_preview_configuration(
                main={"format": "RGB888", "size": (self.width, self.height)}
            )
            self._picam2.configure(cfg)
            self._picam2.start()
            self._mode = "picamera2"
            self.enabled = True
            print("[Vision] Использую PiCamera2.")
            return
        except Exception:
            pass

        # Fallback to OpenCV camera
        try:
            cap = self.cv2.VideoCapture(int(cam_index))
            if cap is not None and cap.isOpened():
                cap.set(self.cv2.CAP_PROP_FRAME_WIDTH, self.width)
                cap.set(self.cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                self._cap = cap
                self._mode = "opencv"
                self.enabled = True
                print("[Vision] Использую OpenCV VideoCapture.")
        except Exception as e:
            print(f"[Vision] Камера недоступна: {e}")
            self.enabled = False

    def close(self) -> None:
        try:
            if self._cap is not None:
                self._cap.release()
        except Exception:
            pass
        try:
            if self._picam2 is not None:
                self._picam2.stop()
        except Exception:
            pass
        if self.debug and self.cv2 is not None:
            try:
                self.cv2.destroyAllWindows()
            except Exception:
                pass

    def _read_bgr(self):
        if not self.enabled:
            return None
        cv2 = self.cv2

        if self._mode == "picamera2" and self._picam2 is not None:
            rgb = self._picam2.capture_array()
            if rgb is None:
                return None
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if self._mode == "opencv" and self._cap is not None:
            ok, frame = self._cap.read()
            return frame if ok else None

        return None

    def _roi_slices(self, h: int, w: int) -> Tuple[slice, slice, slice, slice]:
        y0 = int(h * (1.0 - VISION_ROI_BOTTOM_RATIO))
        roi = slice(y0, h)

        cx0 = int((w * (1.0 - VISION_CENTER_WIDTH_RATIO)) / 2.0)
        cx1 = int(w - cx0)
        center = slice(cx0, cx1)

        left = slice(0, w // 2)
        right = slice(w // 2, w)
        return roi, center, left, right

    def _scores_from_edges(self, bgr) -> Dict[str, Any]:
        cv2 = self.cv2
        np = self.np

        h, w = bgr.shape[:2]
        roi, center, left, right = self._roi_slices(h, w)

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        edges = cv2.Canny(gray, 60, 140)
        if VISION_DILATE_ITERS > 0:
            edges = cv2.dilate(edges, None, iterations=VISION_DILATE_ITERS)

        obs = (edges > 0).astype(np.uint8)
        obs_roi = obs[roi, :]

        def obstacle_frac(region) -> float:
            if region.size == 0:
                return 1.0
            return float(region.mean())

        obs_left = obstacle_frac(obs_roi[:, left])
        obs_right = obstacle_frac(obs_roi[:, right])
        obs_ahead = obstacle_frac(obs_roi[:, center])

        free_left = clamp(1.0 - obs_left, 0.0, 1.0)
        free_right = clamp(1.0 - obs_right, 0.0, 1.0)
        free_ahead = clamp(1.0 - obs_ahead, 0.0, 1.0)

        out = {"free_left": free_left, "free_right": free_right, "free_ahead": free_ahead, "ok": True}

        if self.debug:
            try:
                dbg = bgr.copy()
                y0 = int(h * (1.0 - VISION_ROI_BOTTOM_RATIO))
                cv2.rectangle(dbg, (0, y0), (w - 1, h - 1), (0, 255, 0), 1)
                cx0 = int((w * (1.0 - VISION_CENTER_WIDTH_RATIO)) / 2.0)
                cx1 = int(w - cx0)
                cv2.rectangle(dbg, (cx0, y0), (cx1, h - 1), (255, 0, 0), 1)
                txt = f"free L:{free_left:.2f} R:{free_right:.2f} A:{free_ahead:.2f}"
                cv2.putText(dbg, txt, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                cv2.imshow("smart_camera", dbg)
                cv2.waitKey(1)
            except Exception:
                pass

        return out

    def step(self) -> Optional[Dict[str, Any]]:
        if not self.enabled:
            return None
        t = time.time()
        if (t - self._last_t) < (1.0 / max(self.fps_limit, 1e-3)):
            return self._last_scores
        self._last_t = t

        frame = self._read_bgr()
        if frame is None:
            return None

        scores = self._scores_from_edges(frame)
        self._last_scores = scores
        return scores


class NavigationController:
    def __init__(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        self.kf = RSSIKalmanFilter(R=MEASUREMENT_NOISE_R, Q=PROCESS_NOISE_Q)

        self.vision = SmartCameraAvoider(
            enabled=VISION_ENABLE,
            cam_index=VISION_CAM_INDEX,
            width=VISION_WIDTH,
            height=VISION_HEIGHT,
            fps_limit=VISION_FPS_LIMIT,
            debug=VISION_DEBUG,
        )

        self.state = "SEARCH"  # SEARCH / APPROACH / AVOID / FINISH
        self.last_rssi = -100.0
        self.start_time = time.time()
        self.spiral_angle = -MAX_STEER_ANGLE

        self.center_head()

    # ---------- Управление камерой / головой ----------
    def center_head(self):
        try:
            self.px.set_cam_pan_angle(0)
        except AttributeError:
            pass
        try:
            self.px.set_cam_tilt_angle(0)
        except AttributeError:
            pass

    def set_head_pan(self, angle):
        try:
            self.px.set_cam_pan_angle(angle)
        except AttributeError:
            pass

    # ---------- УЗ сканирование как запасной вариант ----------
    def scan_surroundings(self):
        self.px.stop()
        time.sleep(0.2)

        self.set_head_pan(-45)
        time.sleep(0.3)
        dist_left = self.px.ultrasonic.read()

        self.set_head_pan(45)
        time.sleep(0.3)
        dist_right = self.px.ultrasonic.read()

        self.center_head()
        time.sleep(0.2)

        print(f"👀 Сканирование УЗ: Слева={dist_left}см, Справа={dist_right}см")
        dist_left = dist_left if (dist_left is not None) else 0
        dist_right = dist_right if (dist_right is not None) else 0
        return -1 if dist_left > dist_right else 1

    def run(self):
        print(f"--- 🚀 ЗАПУСК PICARX | Цель: {TARGET_RSSI} dBm | Vision={self.vision.enabled} ---")

        try:
            while True:
                raw_rssi = self.wifi.get_rssi()
                dist = self.px.ultrasonic.read()

                v = self.vision.step()
                rssi = self.kf.filter(raw_rssi) if raw_rssi is not None else self.last_rssi

                if time.time() % 0.5 < 0.05:
                    if v:
                        print(f"[{self.state}] RSSI: {rssi:.1f} dBm | Dist: {dist} см | freeA:{v['free_ahead']:.2f}")
                    else:
                        print(f"[{self.state}] RSSI: {rssi:.1f} dBm | Dist: {dist} см")

                # HARD safety: ultrasonic
                if dist is not None and dist > 0 and dist < OBSTACLE_DIST_CM and self.state != "FINISH":
                    print(f"⛔ ПРЕПЯТСТВИЕ (УЗ {dist} см) — AVOID")
                    self.state = "AVOID"

                # Vision assist (soft safety + steering)
                if v and self.state in ("SEARCH", "APPROACH"):
                    if v["free_ahead"] < VISION_FREE_AHEAD_MIN:
                        print(f"⚠️ Vision: впереди тесно (free_ahead={v['free_ahead']:.2f}) → AVOID")
                        self.state = "AVOID"
                    else:
                        bias = float(v["free_right"] - v["free_left"])
                        steer = VISION_STEER_GAIN * bias * MAX_STEER_ANGLE
                        steer = int(clamp(steer, -MAX_STEER_ANGLE, MAX_STEER_ANGLE))
                        self.px.set_dir_servo_angle(steer)

                if self.state == "AVOID":
                    if v and v.get("ok", False):
                        direction = -1 if v["free_left"] > v["free_right"] else 1
                        print(f"🧠 Vision выбор: {'LEFT' if direction==-1 else 'RIGHT'}")
                    else:
                        direction = self.scan_surroundings()

                    self.px.set_dir_servo_angle(0)
                    self.px.backward(SPEED_SEARCH)
                    time.sleep(0.8)

                    turn_angle = int(direction * MAX_STEER_ANGLE)
                    self.px.set_dir_servo_angle(turn_angle)
                    self.px.forward(SPEED_SEARCH)
                    time.sleep(0.5)

                    self.last_rssi = rssi
                    self.state = "SEARCH"

                elif self.state == "SEARCH":
                    if rssi > APPROACH_THRESHOLD:
                        print(f"✅ Захват сигнала: RSSI={rssi:.1f} dBm → APPROACH")
                        self.state = "APPROACH"
                        self.px.set_dir_servo_angle(0)
                        continue

                    self.px.forward(SPEED_SEARCH)
                    self.px.set_dir_servo_angle(int(self.spiral_angle))
                    if self.spiral_angle < 0:
                        self.spiral_angle += 0.05
                    else:
                        self.spiral_angle = -MAX_STEER_ANGLE

                elif self.state == "APPROACH":
                    if rssi >= TARGET_RSSI:
                        print(f"🎉 Достигнут целевой RSSI {rssi:.1f} dBm → FINISH")
                        self.state = "FINISH"
                        continue

                    delta = rssi - self.last_rssi
                    if delta >= 0:
                        self.px.forward(SPEED_APPROACH)
                    else:
                        self.px.set_dir_servo_angle(20)
                        self.px.forward(SPEED_APPROACH)

                elif self.state == "FINISH":
                    self.px.stop()
                    self.px.set_dir_servo_angle(0)
                    print(f"🎉 ИСТОЧНИК НАЙДЕН! Финальный RSSI: {rssi:.2f} dBm")
                    break

                self.last_rssi = rssi
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n⏹ Остановка по Ctrl+C.")
        except Exception as e:
            print(f"\n💥 Ошибка выполнения: {e}")
        finally:
            try:
                self.px.stop()
                self.px.set_dir_servo_angle(0)
                self.center_head()
            except Exception:
                pass
            try:
                self.vision.close()
            except Exception:
                pass
            print("🧹 Робот остановлен и выровнен.")


if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
