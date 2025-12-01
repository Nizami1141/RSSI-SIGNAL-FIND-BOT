import time
from picarx import Picarx

# ================= КОНФИГУРАЦИЯ =================
TARGET_RSSI = -46        # Цель (dBm) - уровень сигнала рядом с роутером
APPROACH_THRESHOLD = -50  # Порог начала сближения
PROCESS_NOISE_Q = 0.05    # Шум процесса (фильтр Калмана)
MEASUREMENT_NOISE_R = 2.0 # Шум измерений (чем больше, тем плавнее график)
MAX_STEER_ANGLE = 35      # Максимальный угол поворота (защита механики)
OBSTACLE_DIST_CM = 25     # Дистанция до препятствия (см)
SPEED_SEARCH = 40         # Скорость в режиме поиска
SPEED_APPROACH = 30       # Скорость в режиме сближения


# ================= КЛАССЫ =================

class RSSIKalmanFilter:
    """Простой 1D фильтр Калмана для сглаживания RSSI."""
    def __init__(self, R, Q, initial_value=-70):
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
    def __init__(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def get_rssi(self):
        """
        Возвращает RSSI (dBm) как float или None, если не удалось прочитать.

        Формат строки в /proc/net/wireless обычно:
        wlan0: 0000   70.  -51.  -256    ...
                 |     |     |     |
               if    link  level noise
        Нас интересует level (это RSSI), то есть parts[3].
        """
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
                                # На всякий случай очищаем от лишних символов
                                rssi_clean = rssi_raw.replace('.', '')
                                return float(rssi_clean)
        except Exception as e:
            print(f"Ошибка чтения RSSI: {e}")
            return None

        return None


class NavigationController:
    def __init__(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        self.kf = RSSIKalmanFilter(R=MEASUREMENT_NOISE_R, Q=PROCESS_NOISE_Q)
        
        self.state = "SEARCH"          # SEARCH / APPROACH / AVOID / FINISH
        self.last_rssi = -100.0
        self.start_time = time.time()
        self.spiral_angle = -MAX_STEER_ANGLE
        
        self.center_head()

    # ---------- Управление камерой / головой ----------

    def center_head(self):
        """Центруем камеру (пан/тилт), если методы доступны в SDK."""
        try:
            self.px.set_cam_pan_angle(0)
        except AttributeError:
            # На случай, если в твоём SDK нет этой функции
            pass

        try:
            self.px.set_cam_tilt_angle(0)
        except AttributeError:
            pass

    def set_head_pan(self, angle):
        """Поворот камеры влево/вправо (пан)."""
        try:
            self.px.set_cam_pan_angle(angle)
        except AttributeError:
            pass

    # ---------- Логика сканирования ----------

    def scan_surroundings(self):
        """
        Сканирование препятствий слева/справа.
        Возвращает:
            -1 — поворачивать влево
            1  — поворачивать вправо
        """
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
        
        # Назад в центр
        self.center_head()
        time.sleep(0.2)
        
        print(f"👀 Сканирование: Слева={dist_left}см, Справа={dist_right}см")

        if dist_left is None:
            dist_left = 0
        if dist_right is None:
            dist_right = 0

        # Если слева дальше (свободнее) — едем влево, иначе вправо
        return -1 if dist_left > dist_right else 1

    # ---------- Главный цикл ----------

    def run(self):
        print(f"--- 🚀 ЗАПУСК PICARX | Цель: {TARGET_RSSI} dBm ---")
        
        try:
            while True:
                # 1. Чтение датчиков
                raw_rssi = self.wifi.get_rssi()
                dist = self.px.ultrasonic.read()

                # Фильтрация RSSI
                if raw_rssi is not None:
                    rssi = self.kf.filter(raw_rssi)
                else:
                    rssi = self.last_rssi

                # Логирование (примерно каждые 0.5 секунды)
                if time.time() % 0.5 < 0.05:
                    print(f"[{self.state}] RSSI: {rssi:.1f} dBm | Dist: {dist} см")

                # 2. Безопасность: проверка препятствий
                if dist is not None and dist > 0 and dist < OBSTACLE_DIST_CM and self.state != "FINISH":
                    print(f"⛔ ПРЕПЯТСТВИЕ ({dist} см) — ухожу в режим AVOID")
                    self.state = "AVOID"

                # 3. Логика по состояниям
                if self.state == "AVOID":
                    # Сканируем окружение
                    direction = self.scan_surroundings()
                    
                    # Чуть сдаём назад
                    self.px.set_dir_servo_angle(0)
                    self.px.backward(SPEED_SEARCH)
                    time.sleep(0.8)
                    
                    # Поворачиваем и едем вперёд
                    turn_angle = direction * MAX_STEER_ANGLE
                    self.px.set_dir_servo_angle(turn_angle)
                    self.px.forward(SPEED_SEARCH)
                    time.sleep(0.5)
                    
                    self.last_rssi = rssi
                    self.state = "SEARCH"

                elif self.state == "SEARCH":
                    # Если сигнал уже достаточно сильный — переходим к точному сближению
                    if rssi > APPROACH_THRESHOLD:
                        print(f"✅ Захват сигнала: RSSI={rssi:.1f} dBm, перехожу в APPROACH")
                        self.state = "APPROACH"
                        self.px.set_dir_servo_angle(0)
                        continue

                    # Поисковая спираль
                    self.px.forward(SPEED_SEARCH)
                    self.px.set_dir_servo_angle(int(self.spiral_angle))
                    
                    # Спираль постепенно "раскручивается"
                    if self.spiral_angle < 0:
                        self.spiral_angle += 0.05
                    else:
                        self.spiral_angle = -MAX_STEER_ANGLE

                elif self.state == "APPROACH":
                    # Достаточно близко по RSSI — считаем, что нашли источник
                    if rssi >= TARGET_RSSI:
                        print(f"🎉 Достигнут целевой RSSI {rssi:.1f} dBm — перехожу в FINISH")
                        self.state = "FINISH"
                        continue
                    
                    delta = rssi - self.last_rssi

                    # Если сигнал улучшается — едем прямо
                    if delta >= 0:
                        self.px.set_dir_servo_angle(0)
                        self.px.forward(SPEED_APPROACH)
                    else:
                        # Сигнал ухудшается — немного "рыскаем"
                        self.px.set_dir_servo_angle(20)
                        self.px.forward(SPEED_APPROACH)
                        
                elif self.state == "FINISH":
                    self.px.stop()
                    self.px.set_dir_servo_angle(0)
                    print(f"🎉 ИСТОЧНИК НАЙДЕН! Финальный RSSI: {rssi:.2f} dBm")
                    print("⚠️ Замечание: если лог показывает, что я 'нашёл' источник,\n"
                          "но на самом деле ты сам подвёл меня ближе — это ожидаемо 🙂")
                    break

                self.last_rssi = rssi
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n⏹ Остановка по Ctrl+C.")
        except Exception as e:
            print(f"\n💥 Ошибка выполнения: {e}")
        finally:
            # Гарантированная остановка моторов и выравнивание
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            self.center_head()
            print("🧹 Робот остановлен и выровнен.")


if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
