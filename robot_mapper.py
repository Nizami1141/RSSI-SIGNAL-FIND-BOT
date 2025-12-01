import time
import math
import matplotlib
# Указываем бэкенд 'Agg', чтобы не пытаться открыть окно на Raspberry Pi
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from picarx import Picarx

# ================= КОНФИГУРАЦИЯ =================
TARGET_RSSI = -40
APPROACH_THRESHOLD = -65
PROCESS_NOISE_Q = 0.05
MEASUREMENT_NOISE_R = 2.0
MAX_STEER_ANGLE = 35
OBSTACLE_DIST_CM = 25
SPEED_SEARCH = 40
SPEED_APPROACH = 30

# Настройки карты
MAP_FILENAME = 'map.png'
PLOT_EVERY_N_CYCLES = 20  # Обновлять картинку каждые 20 циклов

# ================= КЛАСС ВИЗУАЛИЗАЦИИ =================
class MapVisualizer:
    def __init__(self):
        self.x_data = []
        self.y_data = []
        self.rssi_data = []
        self.fig, self.ax = plt.subplots(figsize=(8, 6))

    def add_point(self, x, y, rssi):
        self.x_data.append(x)
        self.y_data.append(y)
        self.rssi_data.append(rssi)

    def save_map(self):
        if not self.x_data:
            return

        self.ax.clear()
        # Рисуем путь точками, цвет зависит от RSSI
        sc = self.ax.scatter(
            self.x_data,
            self.y_data,
            c=self.rssi_data,
            cmap='jet',
            vmin=-90,
            vmax=-30,
            s=100,
            alpha=0.8
        )

        # Текущее положение робота (черный круг)
        self.ax.plot(self.x_data[-1], self.y_data[-1], 'ko', markersize=10, label="Robot")

        self.ax.set_title(f"Wi-Fi Heatmap (Last RSSI: {self.rssi_data[-1]:.1f} dBm)")
        self.ax.set_xlabel("X (meters approx)")
        self.ax.set_ylabel("Y (meters approx)")
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.axis('equal')

        try:
            plt.savefig(MAP_FILENAME)
        except Exception as e:
            print(f"Ошибка сохранения карты: {e}")

# ================= КЛАССЫ РОБОТА =================

class RSSIKalmanFilter:
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
    def __init__(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def get_rssi(self):
        try:
            with open(self.filepath, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if self.interface + ":" in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            # parts[3] типа "-51."
                            return float(parts[3])
        except Exception as e:
            print(f"Ошибка чтения RSSI: {e}")
            return None
        return None


class NavigationController:
    def __init__(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        self.kf = RSSIKalmanFilter(R=MEASUREMENT_NOISE_R, Q=PROCESS_NOISE_Q)
        self.mapper = MapVisualizer()

        self.state = "SEARCH"
        self.last_rssi = -100
        self.start_time = time.time()
        self.spiral_angle = -MAX_STEER_ANGLE

        # Координаты для карты (Dead Reckoning)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # 0 — "на восток"

        # Для расчёта скорости
        self.current_speed_power = 0
        self.current_steer_angle = 0
        self.last_loop_time = time.time()

        self.center_head()

    # ---------- УПРАВЛЕНИЕ "ГОЛОВОЙ" (КАМЕРОЙ) ----------

    def _set_pan(self, angle):
        """Поворот камеры влево/вправо, если такой функционал есть в SDK."""
        try:
            if hasattr(self.px, "set_cam_pan_angle"):
                self.px.set_cam_pan_angle(angle)
            elif hasattr(self.px, "set_camera_servo1_angle"):
                self.px.set_camera_servo1_angle(angle)
            # если ничего нет — просто игнорируем
        except Exception as e:
            print(f"Ошибка поворота камеры (pan): {e}")

    def _set_tilt(self, angle):
        """Наклон камеры вверх/вниз, если есть в SDK."""
        try:
            if hasattr(self.px, "set_cam_tilt_angle"):
                self.px.set_cam_tilt_angle(angle)
            elif hasattr(self.px, "set_camera_servo2_angle"):
                self.px.set_camera_servo2_angle(angle)
        except Exception as e:
            print(f"Ошибка наклона камеры (tilt): {e}")

    def center_head(self):
        """Центровка камеры, если возможно."""
        self._set_pan(0)
        self._set_tilt(0)

    # ---------- ОДОМЕТРИЯ ----------

    def update_odometry(self):
        current_time = time.time()
        dt = current_time - self.last_loop_time
        self.last_loop_time = current_time

        # Грубая конвертация мощности мотора в м/с (требует калибровки)
        # мощность 100 ≈ 0.4 м/с
        speed_mps = (abs(self.current_speed_power) / 100.0) * 0.4
        if self.current_speed_power < 0:
            speed_mps = -speed_mps

        # Угловая скорость (рад/сек) — тоже очень грубо
        steer_rad = math.radians(self.current_steer_angle)
        angular_velocity = steer_rad * 2.0

        self.theta += angular_velocity * dt
        self.x += speed_mps * math.cos(self.theta) * dt
        self.y += speed_mps * math.sin(self.theta) * dt

    # ---------- ДВИЖЕНИЕ ----------

    def move(self, speed, angle):
        """
        speed > 0  -> ехать вперёд
        speed < 0  -> ехать назад
        """
        self.current_speed_power = speed
        self.current_steer_angle = angle

        self.px.set_dir_servo_angle(angle)

        if speed > 0:
            self.px.forward(speed)
        elif speed < 0:
            self.px.backward(-speed)
        else:
            self.px.stop()

    def stop(self):
        self.current_speed_power = 0
        self.px.stop()

    # ---------- СКАНИРОВАНИЕ ОКРУЖЕНИЯ ----------

    def scan_surroundings(self):
        self.stop()
        time.sleep(0.2)

        self._set_pan(-45)
        time.sleep(0.3)
        dist_left = self.px.ultrasonic.read()

        self._set_pan(45)
        time.sleep(0.3)
        dist_right = self.px.ultrasonic.read()

        self.center_head()
        time.sleep(0.2)

        if dist_left is None:
            dist_left = 0
        if dist_right is None:
            dist_right = 0

        print(f"👀 Сканирование: слева={dist_left} см, справа={dist_right} см")
        return -1 if dist_left > dist_right else 1

    # ---------- ГЛАВНЫЙ ЦИКЛ ----------

    def run(self):
        print(f"--- 🚀 ЗАПУСК | Карта будет в файле {MAP_FILENAME} ---")
        loop_counter = 0

        try:
            while True:
                # 1. Обновление одометрии
                self.update_odometry()

                # 2. Чтение датчиков
                raw_rssi = self.wifi.get_rssi()
                dist = self.px.ultrasonic.read()

                if raw_rssi is not None:
                    rssi = self.kf.filter(raw_rssi)
                else:
                    rssi = self.last_rssi

                # 3. Добавляем точку на карту
                self.mapper.add_point(self.x, self.y, rssi)

                if loop_counter % PLOT_EVERY_N_CYCLES == 0:
                    self.mapper.save_map()
                    print(f"💾 Map updated. Pos=({self.x:.1f}, {self.y:.1f}) RSSI={rssi:.1f} dBm")

                # 4. Избежание препятствий
                if dist is not None and dist > 0 and dist < OBSTACLE_DIST_CM and self.state != "FINISH":
                    print(f"⛔ ПРЕПЯТСТВИЕ: {dist} см")
                    self.state = "AVOID"

                # 5. Логика состояний
                if self.state == "AVOID":
                    direction = self.scan_surroundings()

                    # Назад
                    self.move(-SPEED_SEARCH, 0)
                    time.sleep(0.8)
                    self.update_odometry()

                    # Поворот и вперёд
                    turn_angle = direction * MAX_STEER_ANGLE
                    self.move(SPEED_SEARCH, turn_angle)
                    time.sleep(0.5)
                    self.update_odometry()

                    self.last_rssi = rssi
                    self.state = "SEARCH"

                elif self.state == "SEARCH":
                    if rssi > APPROACH_THRESHOLD:
                        print(f"✅ Захват сигнала (RSSI={rssi:.1f} dBm) -> APPROACH")
                        self.state = "APPROACH"
                        continue

                    self.move(SPEED_SEARCH, int(self.spiral_angle))

                    if self.spiral_angle < 0:
                        self.spiral_angle += 0.05
                    else:
                        self.spiral_angle = -MAX_STEER_ANGLE

                elif self.state == "APPROACH":
                    if rssi >= TARGET_RSSI:
                        print(f"🎯 Достигнут целевой RSSI {rssi:.1f} dBm -> FINISH")
                        self.state = "FINISH"
                        continue

                    delta = rssi - self.last_rssi
                    if delta >= 0:
                        self.move(SPEED_APPROACH, 0)
                    else:
                        self.move(SPEED_APPROACH, 20)

                elif self.state == "FINISH":
                    self.stop()
                    print("🎉 ИСТОЧНИК НАЙДЕН!")
                    self.mapper.save_map()
                    break

                self.last_rssi = rssi
                loop_counter += 1
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nСтоп по Ctrl+C.")
            self.mapper.save_map()
        finally:
            self.stop()
            self.px.set_dir_servo_angle(0)
            self.center_head()
            print("🧹 Робот остановлен и выровнен.")


if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
