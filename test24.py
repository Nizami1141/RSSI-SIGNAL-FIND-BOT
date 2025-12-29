#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PicarX Wi‑Fi seeker + "AI" obstacle avoidance (vision-assisted) with hard ultrasonic safety.

✅ Works even WITHOUT an AI model:
   - Uses ultrasonic for immediate stop/avoid
   - Uses simple vision heuristic to steer away from clutter (optional)

✅ If you provide a TFLite semantic‑segmentation model:
   - Uses segmentation mask to score free space (left/right/ahead)
   - Can treat classes like chair/table/cabinet/window as obstacles

How to run:
  python3 picarx_wifi_ai_avoid.py
Optional:
  python3 picarx_wifi_ai_avoid.py --camera 0 --use_vision 1
  python3 picarx_wifi_ai_avoid.py --model /path/to/seg.tflite --labels /path/to/labels.json

Notes:
- This script is designed to be a "drop‑in replacement" for your test12 logic.
- Safety: always stops motors in finally().
"""

import time
import os
import math
import argparse
from typing import Optional, Dict, Any, Tuple

import numpy as np

# OpenCV is optional (vision can be disabled)
try:
    import cv2  # type: ignore
    _HAVE_CV2 = True
except Exception:
    cv2 = None
    _HAVE_CV2 = False

from picarx import Picarx

# ================= CONFIG (Wi‑Fi + motion) =================
TARGET_RSSI = -46         # (dBm) stop when RSSI >= this
APPROACH_THRESHOLD = -50  # (dBm) switch SEARCH -> APPROACH
PROCESS_NOISE_Q = 0.05    # Kalman Q
MEASUREMENT_NOISE_R = 2.0 # Kalman R
MAX_STEER_ANGLE = 35      # steering safety clamp
OBSTACLE_DIST_CM = 25     # ultrasonic emergency threshold (cm)
SPEED_SEARCH = 40
SPEED_APPROACH = 30

# ================= CONFIG (Vision) =================
DEFAULT_CAM_INDEX = 0
VISION_FPS_LIMIT = 8.0              # limit vision processing rate (Hz)
VISION_ROI_BOTTOM_RATIO = 0.55      # take bottom 55% of image (closest floor area)
VISION_CENTER_WIDTH_RATIO = 0.45    # center "ahead" region width relative to frame
VISION_FREE_AHEAD_MIN = 0.55        # if below -> treat as risky (avoid/slow)
VISION_STEER_GAIN = 1.35            # maps (free_right-free_left) -> steering
VISION_OBS_DILATE = 2               # dilation iterations for heuristic mask

# If using segmentation: these class names are treated as obstacles (customize freely)
DEFAULT_OBSTACLE_CLASS_NAMES = {
    "chair", "table", "sofa", "cabinet", "wardrobe", "dresser", "desk",
    "door", "wall", "window", "shelf", "bed", "stool", "armchair"
}

# ================= Utilities =================

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x

def now_s() -> float:
    return time.time()

# ================= Kalman Filter =================

class RSSIKalmanFilter:
    """Simple 1D Kalman filter for RSSI smoothing."""
    def __init__(self, R: float, Q: float, initial_value: float = -70.0):
        self.R = float(R)
        self.Q = float(Q)
        self.x = float(initial_value)
        self.P = 1.0

    def filter(self, measurement: Optional[float]) -> float:
        if measurement is None:
            return self.x
        z = float(measurement)
        # predict
        p_pred = self.P + self.Q
        # update
        K = p_pred / (p_pred + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = (1.0 - K) * p_pred
        return self.x

# ================= Wi‑Fi Sensor =================

class WiFiSensor:
    """Read RSSI from /proc/net/wireless."""
    def __init__(self, interface: str = "wlan0", filepath: str = "/proc/net/wireless"):
        self.interface = interface
        self.filepath = filepath

    def get_rssi(self) -> Optional[float]:
        try:
            with open(self.filepath, "r") as f:
                for line in f:
                    if (self.interface + ":") in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            # typically parts[3] is "level"
                            raw = parts[3]
                            try:
                                return float(raw)
                            except ValueError:
                                return float(raw.replace(".", ""))
        except Exception as e:
            print(f"Ошибка чтения RSSI: {e}")
        return None

# ================= Camera Provider =================

class CameraProvider:
    """
    Tries PiCamera2 first (if installed), else falls back to OpenCV VideoCapture.
    Returns BGR frames (OpenCV convention) if possible.
    """
    def __init__(self, cam_index: int = 0, width: int = 320, height: int = 240):
        self.cam_index = cam_index
        self.width = width
        self.height = height
        self._mode = "none"
        self._cap = None
        self._picam2 = None
        self._have = False

        # Try PiCamera2
        try:
            from picamera2 import Picamera2  # type: ignore
            self._picam2 = Picamera2()
            config = self._picam2.create_preview_configuration(
                main={"format": "RGB888", "size": (self.width, self.height)}
            )
            self._picam2.configure(config)
            self._picam2.start()
            self._mode = "picamera2"
            self._have = True
            return
        except Exception:
            pass

        # Fallback OpenCV
        if _HAVE_CV2:
            try:
                cap = cv2.VideoCapture(self.cam_index)
                if cap is not None and cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    self._cap = cap
                    self._mode = "opencv"
                    self._have = True
            except Exception:
                self._cap = None
                self._have = False

    def is_ready(self) -> bool:
        return self._have

    def read_bgr(self) -> Optional[np.ndarray]:
        if not self._have:
            return None
        if self._mode == "picamera2" and self._picam2 is not None:
            # PiCamera2 returns RGB
            rgb = self._picam2.capture_array()
            if rgb is None:
                return None
            if _HAVE_CV2:
                return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            # if no cv2, just return RGB but we won't use vision anyway
            return rgb
        if self._mode == "opencv" and self._cap is not None:
            ok, frame = self._cap.read()
            return frame if ok else None
        return None

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

# ================= TFLite Segmentation (optional) =================

class TFLiteSegmentation:
    """
    Minimal TFLite semantic segmentation runner.

    Expected:
      - model input: [1, H, W, 3] uint8 or float32
      - model output: [1, H, W] (class ids) OR [1, H, W, C] (logits/prob)
    """
    def __init__(self, model_path: str, labels_path: Optional[str] = None):
        self.model_path = model_path
        self.labels_path = labels_path
        self._interpreter = None
        self._input_index = None
        self._output_index = None
        self.in_h = None
        self.in_w = None
        self.input_dtype = None
        self.class_name_by_id: Dict[int, str] = {}

        self._load_labels()
        self._load_model()

    def _load_labels(self) -> None:
        if not self.labels_path:
            return
        try:
            with open(self.labels_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            # Accept formats:
            # { "0": "background", "1":"chair", ... } OR { "id_to_name": {...} }
            if isinstance(data, dict) and "id_to_name" in data:
                data = data["id_to_name"]
            if isinstance(data, dict):
                for k, v in data.items():
                    try:
                        self.class_name_by_id[int(k)] = str(v)
                    except Exception:
                        continue
        except Exception as e:
            print(f"[Vision] Не смог прочитать labels.json: {e}")

    def _load_model(self) -> None:
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(self.model_path)

        # Prefer tflite_runtime on Raspberry Pi, fall back to tensorflow if available.
        interpreter_cls = None
        try:
            from tflite_runtime.interpreter import Interpreter  # type: ignore
            interpreter_cls = Interpreter
        except Exception:
            try:
                from tensorflow.lite.python.interpreter import Interpreter  # type: ignore
                interpreter_cls = Interpreter
            except Exception as e:
                raise RuntimeError("Нужен tflite_runtime или tensorflow для TFLite.") from e

        self._interpreter = interpreter_cls(model_path=self.model_path)
        self._interpreter.allocate_tensors()
        inp = self._interpreter.get_input_details()[0]
        out = self._interpreter.get_output_details()[0]
        self._input_index = inp["index"]
        self._output_index = out["index"]

        shape = inp["shape"]  # [1,H,W,3]
        self.in_h = int(shape[1])
        self.in_w = int(shape[2])
        self.input_dtype = inp["dtype"]

    def infer_class_ids(self, bgr_frame: np.ndarray) -> np.ndarray:
        if self._interpreter is None:
            raise RuntimeError("Interpreter not loaded.")
        assert self.in_h is not None and self.in_w is not None

        # resize
        if _HAVE_CV2:
            img = cv2.resize(bgr_frame, (self.in_w, self.in_h), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        else:
            # minimal fallback
            rgb = bgr_frame

        x = rgb
        if self.input_dtype == np.float32:
            x = x.astype(np.float32) / 255.0
        else:
            x = x.astype(self.input_dtype)

        x = np.expand_dims(x, axis=0)
        self._interpreter.set_tensor(self._input_index, x)
        self._interpreter.invoke()
        y = self._interpreter.get_tensor(self._output_index)

        # y could be [1,H,W] or [1,H,W,C]
        y = np.squeeze(y, axis=0)
        if y.ndim == 2:
            class_ids = y.astype(np.int32)
        elif y.ndim == 3:
            class_ids = np.argmax(y, axis=-1).astype(np.int32)
        else:
            raise RuntimeError(f"Unexpected output shape: {y.shape}")
        return class_ids

# ================= Vision Scoring =================

class VisionObstacleScorer:
    """
    Converts either:
      - segmentation class_id map, OR
      - heuristic edge-based obstacle mask
    into free-space scores: free_left/free_right/free_ahead in [0..1].
    """
    def __init__(
        self,
        obstacle_class_names = None,
        roi_bottom_ratio: float = VISION_ROI_BOTTOM_RATIO,
        center_width_ratio: float = VISION_CENTER_WIDTH_RATIO,
        debug: bool = False,
    ):
        self.roi_bottom_ratio = float(roi_bottom_ratio)
        self.center_width_ratio = float(center_width_ratio)
        self.debug = bool(debug)
        self.obstacle_class_names = set(obstacle_class_names or DEFAULT_OBSTACLE_CLASS_NAMES)

    def _roi_slices(self, h: int, w: int) -> Tuple[slice, slice, slice]:
        y0 = int(h * (1.0 - self.roi_bottom_ratio))
        roi = slice(y0, h)
        cx0 = int((w * (1.0 - self.center_width_ratio)) / 2.0)
        cx1 = int(w - cx0)
        center = slice(cx0, cx1)
        left = slice(0, w // 2)
        right = slice(w // 2, w)
        return roi, center, left, right

    def _scores_from_obstacle_mask(self, obs: np.ndarray) -> Dict[str, float]:
        # obs is boolean or {0,1}
        h, w = obs.shape[:2]
        roi, center, left, right = self._roi_slices(h, w)

        obs_roi = obs[roi, :]
        # fractions of obstacle pixels in each region
        def frac(region: np.ndarray) -> float:
            if region.size == 0:
                return 1.0
            return float(np.mean(region.astype(np.float32)))

        obs_left = frac(obs_roi[:, left])
        obs_right = frac(obs_roi[:, right])
        obs_ahead = frac(obs_roi[:, center])

        free_left = clamp(1.0 - obs_left, 0.0, 1.0)
        free_right = clamp(1.0 - obs_right, 0.0, 1.0)
        free_ahead = clamp(1.0 - obs_ahead, 0.0, 1.0)

        return {"free_left": free_left, "free_right": free_right, "free_ahead": free_ahead}

    def obstacle_from_seg(
        self,
        class_ids: np.ndarray,
        class_name_by_id: Dict[int, str]
    ) -> Tuple[np.ndarray, bool]:
        # Build obstacle mask from class names
        obs = np.zeros_like(class_ids, dtype=np.uint8)
        window_risk = False
        for cid in np.unique(class_ids):
            name = class_name_by_id.get(int(cid), "")
            if name in self.obstacle_class_names:
                obs[class_ids == cid] = 1
                if name == "window":
                    window_risk = True
        return obs.astype(bool), window_risk

    def obstacle_heuristic(self, bgr: np.ndarray) -> np.ndarray:
        """
        Heuristic obstacle detector:
          - edges / vertical structures in bottom ROI are treated as obstacles.
        This is NOT semantic, but works as a lightweight fallback.
        """
        if not _HAVE_CV2:
            return np.zeros((bgr.shape[0], bgr.shape[1]), dtype=bool)

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(gray, 60, 140)
        # thicken edges
        if VISION_OBS_DILATE > 0:
            edges = cv2.dilate(edges, None, iterations=VISION_OBS_DILATE)
        # treat dense edges as obstacles
        obs = edges > 0
        return obs

    def score(
        self,
        bgr: np.ndarray,
        seg_class_ids: Optional[np.ndarray] = None,
        class_name_by_id: Optional[Dict[int, str]] = None
    ) -> Dict[str, Any]:
        window_risk = False
        if seg_class_ids is not None and class_name_by_id is not None:
            obs, window_risk = self.obstacle_from_seg(seg_class_ids, class_name_by_id)
        else:
            obs = self.obstacle_heuristic(bgr)

        scores = self._scores_from_obstacle_mask(obs)
        scores["window_risk"] = bool(window_risk)
        scores["ok"] = True
        return scores

# ================= Navigation Controller =================

class NavigationController:
    def __init__(
        self,
        use_vision: bool = True,
        cam_index: int = DEFAULT_CAM_INDEX,
        tflite_model: Optional[str] = None,
        labels_json: Optional[str] = None,
        show_debug: bool = False,
    ):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        self.kf = RSSIKalmanFilter(R=MEASUREMENT_NOISE_R, Q=PROCESS_NOISE_Q)

        self.state = "SEARCH"  # SEARCH / APPROACH / AVOID / FINISH
        self.last_rssi = -100.0
        self.start_time = now_s()
        self.spiral_angle = -MAX_STEER_ANGLE

        self.use_vision = bool(use_vision) and _HAVE_CV2
        self.show_debug = bool(show_debug) and _HAVE_CV2

        self.camera = CameraProvider(cam_index=cam_index) if self.use_vision else None
        self.seg = None
        self.scorer = VisionObstacleScorer(debug=self.show_debug)

        if self.use_vision and self.camera and not self.camera.is_ready():
            print("[Vision] Камера не доступна. Продолжаю без vision.")
            self.use_vision = False
            self.camera = None

        if self.use_vision and tflite_model:
            try:
                self.seg = TFLiteSegmentation(tflite_model, labels_json)
                print(f"[Vision] TFLite модель загружена: {tflite_model}")
            except Exception as e:
                print(f"[Vision] Не смог загрузить TFLite модель: {e}")
                self.seg = None

        self._last_vision_t = 0.0
        self._last_scores: Optional[Dict[str, Any]] = None

        self.center_head()

    # ---------- Camera head ----------
    def center_head(self) -> None:
        try:
            self.px.set_cam_pan_angle(0)
        except AttributeError:
            pass
        try:
            self.px.set_cam_tilt_angle(0)
        except AttributeError:
            pass

    def set_head_pan(self, angle: int) -> None:
        try:
            self.px.set_cam_pan_angle(angle)
        except AttributeError:
            pass

    # ---------- Ultrasonic scan ----------
    def scan_surroundings(self) -> int:
        """Ultrasonic scan: returns -1 (turn left) or +1 (turn right)."""
        self.px.stop()
        time.sleep(0.2)

        self.set_head_pan(-45)
        time.sleep(0.25)
        dist_left = self.px.ultrasonic.read()

        self.set_head_pan(45)
        time.sleep(0.25)
        dist_right = self.px.ultrasonic.read()

        self.center_head()
        time.sleep(0.1)

        if dist_left is None: dist_left = 0
        if dist_right is None: dist_right = 0

        print(f"👀 УЗ-скан: слева={dist_left}см, справа={dist_right}см")
        return -1 if dist_left > dist_right else 1

    # ---------- Vision step ----------
    def vision_step(self) -> Optional[Dict[str, Any]]:
        if not self.use_vision or self.camera is None:
            return None

        # rate limit
        t = now_s()
        if (t - self._last_vision_t) < (1.0 / max(VISION_FPS_LIMIT, 1e-3)):
            return self._last_scores
        self._last_vision_t = t

        frame = self.camera.read_bgr()
        if frame is None:
            return None

        seg_ids = None
        class_name_by_id = None
        if self.seg is not None:
            try:
                seg_ids = self.seg.infer_class_ids(frame)
                class_name_by_id = self.seg.class_name_by_id
            except Exception as e:
                # if model fails, fall back to heuristic
                print(f"[Vision] Ошибка инференса: {e}")
                seg_ids = None
                class_name_by_id = None

        scores = self.scorer.score(frame, seg_ids, class_name_by_id)

        if self.show_debug:
            try:
                # draw debug overlay
                h, w = frame.shape[:2]
                y0 = int(h * (1.0 - VISION_ROI_BOTTOM_RATIO))
                cx0 = int((w * (1.0 - VISION_CENTER_WIDTH_RATIO)) / 2.0)
                cx1 = int(w - cx0)
                dbg = frame.copy()
                cv2.rectangle(dbg, (0, y0), (w - 1, h - 1), (0, 255, 0), 1)
                cv2.rectangle(dbg, (cx0, y0), (cx1, h - 1), (255, 0, 0), 1)
                txt = f"L:{scores['free_left']:.2f} R:{scores['free_right']:.2f} A:{scores['free_ahead']:.2f}"
                cv2.putText(dbg, txt, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                cv2.imshow("vision_debug", dbg)
                cv2.waitKey(1)
            except Exception:
                pass

        self._last_scores = scores
        return scores

    # ---------- Main loop ----------
    def run(self) -> None:
        print(f"--- 🚀 PICARX | цель RSSI: {TARGET_RSSI} dBm | vision={self.use_vision} ---")

        try:
            while True:
                # 1) Sensors
                raw_rssi = self.wifi.get_rssi()
                dist_cm = self.px.ultrasonic.read()

                # RSSI smooth
                rssi = self.kf.filter(raw_rssi) if raw_rssi is not None else self.last_rssi

                # Vision scores (optional)
                scores = self.vision_step()

                # Light logging ~2 Hz
                if now_s() % 0.5 < 0.05:
                    if scores:
                        print(f"[{self.state}] RSSI:{rssi:.1f} dBm | Dist:{dist_cm}cm | freeA:{scores['free_ahead']:.2f}")
                    else:
                        print(f"[{self.state}] RSSI:{rssi:.1f} dBm | Dist:{dist_cm}cm")

                # 2) Hard safety (ultrasonic)
                if dist_cm is not None and dist_cm > 0 and dist_cm < OBSTACLE_DIST_CM and self.state != "FINISH":
                    print(f"⛔ УЗ-препятствие ({dist_cm}см) → AVOID")
                    self.state = "AVOID"

                # 3) Soft safety / steering from vision (if not in FINISH/AVOID)
                if scores and self.state in ("SEARCH", "APPROACH"):
                    # if ahead is too blocked, trigger avoid earlier
                    if (scores["free_ahead"] < VISION_FREE_AHEAD_MIN) or scores.get("window_risk", False):
                        print(f"⚠️ Vision риск (free_ahead={scores['free_ahead']:.2f}, window={scores.get('window_risk')}) → AVOID")
                        self.state = "AVOID"
                    else:
                        # steer gently to freer side
                        bias = float(scores["free_right"] - scores["free_left"])  # + => right freer
                        steer = VISION_STEER_GAIN * bias * MAX_STEER_ANGLE
                        steer = int(clamp(steer, -MAX_STEER_ANGLE, MAX_STEER_ANGLE))
                        self.px.set_dir_servo_angle(steer)

                # 4) State machine
                if self.state == "AVOID":
                    # Decide turn direction: use vision if available, else ultrasonic scan
                    direction = None
                    if scores:
                        direction = -1 if scores["free_left"] > scores["free_right"] else 1
                    else:
                        direction = self.scan_surroundings()

                    # back up
                    self.px.set_dir_servo_angle(0)
                    self.px.backward(SPEED_SEARCH)
                    time.sleep(0.8)

                    # turn + forward
                    turn_angle = int(direction * MAX_STEER_ANGLE)
                    self.px.set_dir_servo_angle(turn_angle)
                    self.px.forward(SPEED_SEARCH)
                    time.sleep(0.6)

                    # return to SEARCH
                    self.last_rssi = rssi
                    self.state = "SEARCH"

                elif self.state == "SEARCH":
                    if rssi > APPROACH_THRESHOLD:
                        print(f"✅ Захват сигнала: RSSI={rssi:.1f} → APPROACH")
                        self.state = "APPROACH"
                        self.px.set_dir_servo_angle(0)
                        continue

                    # spiral search motion
                    self.px.forward(SPEED_SEARCH)
                    self.px.set_dir_servo_angle(int(self.spiral_angle))
                    if self.spiral_angle < 0:
                        self.spiral_angle += 0.05
                    else:
                        self.spiral_angle = -MAX_STEER_ANGLE

                elif self.state == "APPROACH":
                    if rssi >= TARGET_RSSI:
                        print(f"🎉 Целевой RSSI достигнут: {rssi:.1f} → FINISH")
                        self.state = "FINISH"
                        continue

                    delta = rssi - self.last_rssi
                    if delta >= 0:
                        # improving: go straighter (vision may still override angle slightly)
                        self.px.forward(SPEED_APPROACH)
                    else:
                        # worsening: small wiggle
                        self.px.set_dir_servo_angle(int(clamp(20, -MAX_STEER_ANGLE, MAX_STEER_ANGLE)))
                        self.px.forward(SPEED_APPROACH)

                elif self.state == "FINISH":
                    self.px.stop()
                    self.px.set_dir_servo_angle(0)
                    print(f"🎉 ИСТОЧНИК НАЙДЕН. Финальный RSSI: {rssi:.2f} dBm")
                    break

                self.last_rssi = rssi
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n⏹ Остановка по Ctrl+C.")
        except Exception as e:
            print(f"\n💥 Ошибка выполнения: {e}")
        finally:
            # Guaranteed safe stop
            try:
                self.px.stop()
                self.px.set_dir_servo_angle(0)
                self.center_head()
            except Exception:
                pass
            try:
                if self.camera:
                    self.camera.close()
            except Exception:
                pass
            if self.show_debug and _HAVE_CV2:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass
            print("🧹 Робот остановлен и выровнен.")

# ================= Entry =================

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--use_vision", type=int, default=1, help="1=use camera vision, 0=disable")
    ap.add_argument("--camera", type=int, default=DEFAULT_CAM_INDEX, help="camera index for OpenCV")
    ap.add_argument("--model", type=str, default="", help="path to TFLite segmentation model (.tflite)")
    ap.add_argument("--labels", type=str, default="", help="path to labels.json mapping id->name")
    ap.add_argument("--debug_view", type=int, default=0, help="1=show OpenCV debug window")
    args = ap.parse_args()

    model = args.model.strip() or None
    labels = args.labels.strip() or None

    ctrl = NavigationController(
        use_vision=bool(args.use_vision),
        cam_index=int(args.camera),
        tflite_model=model,
        labels_json=labels,
        show_debug=bool(args.debug_view),
    )
    ctrl.run()

if __name__ == "__main__":
    main()
