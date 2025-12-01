import time
from picarx import Picarx
import random

# ================= КОНФИГУРАЦИЯ =================
TARGET_RSSI = -50         # Цель (dBm) - Понизил до -50 (реалистичнее)
APPROACH_THRESHOLD = -65  # Порог начала сближения
PROCESS_NOISE_Q = 0.05    # Шум процесса (фильтр Калмана)
MEASUREMENT_NOISE_R = 2.0 # Шум измерений 
MAX_STEER_ANGLE = 35      # Макс угол поворота
OBSTACLE_DIST_CM = 25     # Дистанция до препятствия
SPEED_SEARCH = 40         # Скорость поиска
SPEED_APPROACH = 30       # Скорость сближения


# ================= КЛАССЫ =================

class RSSIKalmanFilter:
    """Простой 1D фильтр Калмана для сглаживания RSSI."""
    def _init_(self, R, Q, initial_value=-70):
        self.R = R          # шум измерения
        self.Q = Q          # шум процесса
        self.x = initial_value  # апостериорная оценка
        self.P = 1.0            # апостериорная дисперсия

    def filter(self, measurement):
        """Возвращает сглаженное значение RSSI."""
        if measurement is None:
            return self.x
        
        # Предикция
        p_pred = self.P + self.Q
        
        # Обновление
        K = p_pred / (p_pred + self.R)      # коэффициент Калмана
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * p_pred
        return self.x


class WiFiSensor:
    """Чтение уровня RSSI из /proc/net/wireless."""
    def _init_(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def get_rssi(self):
        """Возвращает RSSI (dBm) как float или None."""
        try:
            with open(self.filepath, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if self.interface + ":" in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            rssi_raw = parts[3]  # например "-51."
                            try:
                                rssi = float(rssi_raw)
                                return rssi
                            except ValueError:
                                rssi_clean = rssi_raw.replace('.', '')
                                return float(rssi_clean)
        except Exception:
            return None
        return None


class NavigationController:
    def _init_(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        self.kf = RSSIKalmanFilter(R=MEASUREMENT_NOISE_R, Q=PROCESS_NOISE_Q)
        
        self.state = "SEARCH"           # SEARCH / APPROACH / AVOID / FINISH
        self.last_rssi = -100.0
        self.prev_raw_rssi = -999.0     # Для проверки обновления данных
        self.spiral_angle = -MAX_STEER_ANGLE
        
        self.center_head()

    # ---------- Управление камерой ----------

    def center_head(self):
        try:
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except AttributeError:
            pass

    def set_head_pan(self, angle):
        try:
            self.px.set_cam_pan_angle(angle)
        except AttributeError:
            pass

    # ---------- Логика сканирования ----------

    def scan_surroundings(self):
        """Сканирование препятствий для объезда."""
        self.px.stop()
        time.sleep(0.2)
        
        # Влево
        self.set_head_pan(-45)
        time.sleep(0.3)
        dist_left = self.px.ultrasonic.read()
        
        # Вправо
        self.set_head_pan(45)
        time.sleep(0.3)
        dist_right = self.px.ultrasonic.read()
        
        self.center_head()
        time.sleep(0.2)

        if dist_left is None: dist_left = 0
        if dist_right is None: dist_right = 0

        # Возвращаем направление: -1 (влево), 1 (вправо)
        return -1 if dist_left > dist_right else 1

    # ---------- Главный цикл ----------

    def run(self):
        print(f"--- 🚀 ЗАПУСК PICARX | Цель: {TARGET_RSSI} dBm ---")
        
        try:
            while True:
                # 1. Чтение Ультразвука (Быстрое обновление)
                dist = self.px.ultrasonic.read()

                # 2. Безопасность (проверяем всегда, даже если Wi-Fi тупит)
                if dist is not None and 0 < dist < OBSTACLE_DIST_CM and self.state != "FINISH":
                    print(f"⛔ ПРЕПЯТСТВИЕ ({dist} см) — AVOID")
                    self.state = "AVOID"

                # 3. Чтение RSSI (Может быть старым/stale)
                raw_rssi = self.wifi.get_rssi()
                
                # ==== ФИЛЬТРАЦИЯ ЛАГА WI-FI ====
                # Если данные не изменились с прошлого раза, мы не пересчитываем логику навигации,
                # просто продолжаем выполнять последнее действие (ехать).
                if raw_rssi is not None and raw_rssi == self.prev_raw_rssi:
                    time.sleep(0.05) # Небольшая пауза
                    continue 
                
                # Данные обновились! Запоминаем новые
                self.prev_raw_rssi = raw_rssi 
                
                # Фильтруем Калманом
                if raw_rssi is not None:
                    rssi = self.kf.filter(raw_rssi)
                else:
                    rssi = self.last_rssi

                print(f"[{self.state}] RSSI: {rssi:.1f} dBm (Raw: {raw_rssi})")

                # 4. Логика по состояниям
                if self.state == "AVOID":
                    direction = self.scan_surroundings()
                    
                    self.px.set_dir_servo_angle(0)
                    self.px.backward(SPEED_SEARCH)
                    time.sleep(0.8)
                    
                    turn_angle = direction * MAX_STEER_ANGLE
                    self.px.set_dir_servo_angle(turn_angle)
                    self.px.forward(SPEED_SEARCH)
                    time.sleep(0.6)
                    
                    self.last_rssi = rssi
                    self.state = "SEARCH"

                elif self.state == "SEARCH":
                    if rssi > APPROACH_THRESHOLD:
                        print(f"✅ Сигнал найден ({rssi:.1f}), переход в APPROACH")
                        self.state = "APPROACH"
                        self.px.set_dir_servo_angle(0)
                        continue

                    # Движение "Змейкой" (лучше чем спираль для Wi-Fi)
                    # Каждые 2 секунды меняем направление чуть-чуть
                    wiggle = 20 if (time.time() % 4 < 2) else -20
                    self.px.set_dir_servo_angle(wiggle)
                    self.px.forward(SPEED_SEARCH)

                elif self.state == "APPROACH":
                    if rssi >= TARGET_RSSI:
                        print(f"🎉 ФИНИШ! Цель достигнута: {rssi:.1f} dBm")
                        self.state = "FINISH"
                        continue
                    
                    delta = rssi - self.last_rssi

                    if delta >= 0:
                        # Сигнал растет или такой же - едем прямо
                        self.px.set_dir_servo_angle(0)
                        self.px.forward(SPEED_APPROACH)
                    else:
                        # Сигнал падает! Мы едем не туда.
                        # Делаем резкий корректирующий поворот
                        print("  🔻 Сигнал падает, коррекция курса...")
                        # Случайный выбор стороны, чтобы не зациклиться
                        correction = random.choice([-30, 30])
                        self.px.set_dir_servo_angle(correction)
                        self.px.forward(SPEED_APPROACH)
                        # Едем чуть дольше, чтобы выйти из зоны плохого сигнала
                        time.sleep(0.5) 
                        
                elif self.state == "FINISH":
                    self.px.stop()
                    self.px.set_dir_servo_angle(0)
                    break

                self.last_rssi = rssi
                # Пауза не нужна, так как мы уже ждем обновления данных в начале цикла

        except KeyboardInterrupt:
            print("\n⏹ Остановка.")
        except Exception as e:
            print(f"\n💥 Ошибка: {e}")
        finally:
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            self.center_head()

if _name_ == "_main_":
    bot = NavigationController()
    bot.run()
