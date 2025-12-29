import time
from picarx import Picarx

# ================= КОНФИГУРАЦИЯ =================
TARGET_RSSI = -46         # Цель (dBm)
APPROACH_THRESHOLD = -50  # Порог сближения
PROCESS_NOISE_Q = 0.05    # Шум процесса (Калман)
MEASUREMENT_NOISE_R = 2.0 # Шум измерений (Калман)
MAX_STEER_ANGLE = 35
OBSTACLE_DIST_CM = 25
SPEED_SEARCH = 40
SPEED_APPROACH = 30

# Настройки стабильности
OBSTACLE_CONFIRM_COUNT = 3 # Фильтр препятствий (сколько раз увидеть стену)
VERIFY_DURATION = 20       # Время проверки финиша (сек)
VERIFY_REQUIRED_HITS = 4   # Количество подтверждений цели

# Настройки пакетного чтения
BURST_SAMPLES = 5          # Сколько делать микро-замеров за один цикл


# ================= КЛАССЫ =================

class RSSIKalmanFilter:
    """Фильтр Калмана. Работает ПОВЕРХ пакетного чтения для идеальной плавности."""
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

    def _read_single_rssi(self):
        """Внутренний метод: однократное чтение файла."""
        try:
            with open(self.filepath, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if self.interface + ":" in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            rssi_raw = parts[3].replace('.', '')
                            return float(rssi_raw)
        except Exception:
            return None
        return None

    def get_stable_rssi(self, samples=5):
        """
        🔥 ПАКЕТНОЕ ЧТЕНИЕ (Burst Sampling)
        Делает несколько замеров, удаляет выбросы (min/max) и усредняет.
        """
        readings = []
        
        # 1. Сбор данных
        for _ in range(samples):
            val = self._read_single_rssi()
            if val is not None:
                readings.append(val)
            # Микро-пауза, чтобы ОС успела обновить буфер (хотя бы виртуально)
            time.sleep(0.02) 

        if not readings:
            return None

        # 2. Фильтрация выбросов (если набрали хотя бы 3 значения)
        if len(readings) >= 3:
            readings.sort()
            # Удаляем самый маленький и самый большой результат (шумы)
            readings = readings[1:-1]

        if not readings: # Если после обрезки пусто (например, было всего 1-2 замера)
            return None

        # 3. Усреднение
        average_rssi = sum(readings) / len(readings)
        return average_rssi


class NavigationController:
    def __init__(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        self.kf = RSSIKalmanFilter(R=MEASUREMENT_NOISE_R, Q=PROCESS_NOISE_Q)
        
        self.state = "SEARCH"
        self.last_rssi = -100.0
        self.spiral_angle = -MAX_STEER_ANGLE
        
        # Переменные логики
        self.obstacle_counter = 0 
        self.verify_start_time = 0
        self.verify_hits = 0

        self.center_head()

    # ---------- Управление ----------
    def center_head(self):
        try:
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except AttributeError: pass

    def set_head_pan(self, angle):
        try:
            self.px.set_cam_pan_angle(angle)
        except AttributeError: pass

    def scan_surroundings(self):
        self.px.stop()
        time.sleep(0.2)
        
        self.set_head_pan(-45)
        time.sleep(0.3)
        left = self.px.ultrasonic.read()
        
        self.set_head_pan(45)
        time.sleep(0.3)
        right = self.px.ultrasonic.read()
        
        self.center_head()
        if left is None: left = 0
        if right is None: right = 0
        
        print(f"👀 Скан: L={left}, R={right}")
        return -1 if left > right else 1

    # ---------- Главный цикл ----------
    def run(self):
        print(f"--- 🚀 ЗАПУСК PICARX (Burst Mode) | Цель: {TARGET_RSSI} ---")
        
        # Инициализация фильтра первым стабильным замером
        start_val = self.wifi.get_stable_rssi(samples=10)
        if start_val:
            self.kf.x = start_val

        try:
            while True:
                # 1. Чтение датчиков (ТЕПЕРЬ ИСПОЛЬЗУЕМ STABLE RSSI)
                # Берем 5 семплов за раз. Это займет около 0.1 сек (5 * 0.02)
                raw_rssi = self.wifi.get_stable_rssi(samples=BURST_SAMPLES)
                dist = self.px.ultrasonic.read()

                # Калман сглаживает уже усредненное значение
                if raw_rssi is not None:
                    rssi = self.kf.filter(raw_rssi)
                else:
                    rssi = self.last_rssi

                # Лог
                if time.time() % 0.5 < 0.1:
                    print(f"[{self.state}] RSSI: {rssi:.1f} | Hits: {self.verify_hits} | Obs: {self.obstacle_counter}")

                # 2. Фильтр препятствий (Debounce)
                if self.state not in ["FINISH", "VERIFY"]:
                    if dist is not None and dist > 0 and dist < OBSTACLE_DIST_CM:
                        self.obstacle_counter += 1
                    else:
                        self.obstacle_counter = 0 
                    
                    if self.obstacle_counter >= OBSTACLE_CONFIRM_COUNT:
                        print(f"⛔ ПРЕПЯТСТВИЕ ПОДТВЕРЖДЕНО -> AVOID")
                        self.state = "AVOID"
                        self.obstacle_counter = 0

                # 3. Логика состояний
                if self.state == "AVOID":
                    direction = self.scan_surroundings()
                    self.px.set_dir_servo_angle(0)
                    self.px.backward(SPEED_SEARCH)
                    time.sleep(0.8)
                    
                    turn = direction * MAX_STEER_ANGLE
                    self.px.set_dir_servo_angle(turn)
                    self.px.forward(SPEED_SEARCH)
                    time.sleep(0.5)
                    self.last_rssi = rssi
                    self.state = "SEARCH"

                elif self.state == "SEARCH":
                    if rssi > APPROACH_THRESHOLD:
                        print(f"✅ Сигнал пойман ({rssi:.1f}) -> APPROACH")
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
                    # Попали в зону цели?
                    if rssi >= TARGET_RSSI:
                        print(f"🧐 Подозрение на цель ({rssi:.1f}). Проверка...")
                        self.state = "VERIFY"
                        self.px.stop()
                        self.px.set_dir_servo_angle(0)
                        self.verify_start_time = time.time()
                        self.verify_hits = 0
                        continue
                    
                    delta = rssi - self.last_rssi
                    if delta >= 0:
                        self.px.set_dir_servo_angle(0)
                        self.px.forward(SPEED_APPROACH)
                    else:
                        self.px.set_dir_servo_angle(20)
                        self.px.forward(SPEED_APPROACH)

                elif self.state == "VERIFY":
                    elapsed = time.time() - self.verify_start_time
                    
                    # В режиме проверки можно делать даже больше семплов для точности
                    # Но пока используем те же, что в основном цикле
                    if rssi >= TARGET_RSSI:
                        self.verify_hits += 1
                        print(f"   👍 Hit! {self.verify_hits}/{VERIFY_REQUIRED_HITS}")

                    if elapsed > VERIFY_DURATION:
                        if self.verify_hits >= VERIFY_REQUIRED_HITS:
                            print(f"🎉 ФИНИШ! ({self.verify_hits} hits)")
                            self.state = "FINISH"
                        else:
                            print(f"❌ Ложная тревога. Назад.")
                            self.px.backward(SPEED_APPROACH)
                            time.sleep(1.0)
                            self.state = "APPROACH"

                elif self.state == "FINISH":
                    self.px.stop()
                    break

                self.last_rssi = rssi
                # Важно: пауза цикла может быть меньше, так как задержка уже есть внутри get_stable_rssi
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n⏹ Остановка.")
        finally:
            self.px.stop()
            self.px.set_dir_servo_angle(0)
            self.center_head()

if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
