import time
import statistics
from picarx import Picarx

# ================= КОНФИГУРАЦИЯ =================
TARGET_RSSI = -46         # Цель (dBm)
APPROACH_THRESHOLD = -55  # Порог, когда начинаем "прицеливаться"

# Настройки выборки (Твой запрос)
BURST_SAMPLES = 15        # Брем 15 сигналов за один раз
SAMPLE_DELAY = 0.01       # Пауза между микро-замерами

# Настройки логики
OBSTACLE_DIST_CM = 25
SPEED_SEARCH = 40
SPEED_APPROACH = 30
VERIFY_DURATION = 20      # Секунд ждать на финише для подтверждения
VERIFY_REQUIRED_HITS = 4  # Сколько раз сигнал должен быть идеальным

# ================= КЛАСС WIFI (УМНОЕ ЧТЕНИЕ) =================

class WiFiSensor:
    def __init__(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def _read_raw(self):
        """Читает одно сырое значение из файла."""
        try:
            with open(self.filepath, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if self.interface + ":" in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            val = parts[3].replace('.', '')
                            return float(val)
        except:
            pass
        return None

    def get_averaged_rssi(self, count=15):
        """
        🔥 БЕРЕТ 15 СИГНАЛОВ, ЧИСТИТ И УСРЕДНЯЕТ
        """
        readings = []
        for _ in range(count):
            val = self._read_raw()
            if val is not None:
                readings.append(val)
            time.sleep(SAMPLE_DELAY) # Микро-пауза
        
        if not readings:
            return None
        
        # Если набрали достаточно данных (хотя бы 5), убираем выбросы
        # Это удаляет случайные резкие скачки шума
        if len(readings) >= 5:
            readings.sort()
            # Убираем 20% данных с краев (самые маленькие и самые большие)
            trim_amount = int(len(readings) * 0.2)
            if trim_amount > 0:
                readings = readings[trim_amount:-trim_amount]
            
        if not readings: return None

        return statistics.mean(readings)


# ================= ГЛАВНЫЙ КОНТРОЛЛЕР =================

class NavigationController:
    def __init__(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        
        self.state = "SEARCH"
        self.prev_avg_rssi = -100.0 # Хранит ПРОШЛОЕ среднее значение (из 15)
        
        # Переменные для логики
        self.spiral_angle = -35
        self.consecutive_drops = 0  # Счетчик ухудшения сигнала
        self.obstacle_counter = 0   # Счетчик препятствий
        
        # Переменные для финиша
        self.verify_start_time = 0
        self.verify_hits = 0

        self.center_head()

    # --- Утилиты ---
    def center_head(self):
        try:
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except: pass

    def scan_surroundings(self):
        """Сканирует, куда лучше повернуть при препятствии."""
        self.px.stop()
        self.px.set_cam_pan_angle(-45)
        time.sleep(0.3)
        left = self.px.ultrasonic.read()
        self.px.set_cam_pan_angle(45)
        time.sleep(0.3)
        right = self.px.ultrasonic.read()
        self.center_head()
        
        if left is None: left = 0
        if right is None: right = 0
        return -1 if left > right else 1

    # --- MAIN LOOP ---
    def run(self):
        print(f"--- 🚀 ЗАПУСК (Logic v2.5) | Цель: {TARGET_RSSI} ---")
        
        # 1. Первый замер для калибровки "прошлого" значения
        print("📊 Калибровка стартового сигнала (ждем 0.5с)...")
        start_val = self.wifi.get_averaged_rssi(BURST_SAMPLES)
        if start_val: 
            self.prev_avg_rssi = start_val
            print(f"📊 Старт с RSSI: {start_val:.1f}")
        
        try:
            while True:
                # ---------------------------------------------------------
                # ШАГ 1: Читаем "Пакет" (15 сигналов)
                # Это займет примерно: 15 * 0.01 = 0.15 сек
                curr_rssi = self.wifi.get_averaged_rssi(BURST_SAMPLES)
                dist = self.px.ultrasonic.read()

                if curr_rssi is None:
                    continue # Если ошибка чтения, пропускаем цикл

                # ---------------------------------------------------------
                # ШАГ 2: Сравниваем с прошлым (Тренд)
                rssi_delta = curr_rssi - self.prev_avg_rssi
                trend_icon = "⬆️" if rssi_delta >= 0 else "⬇️"
                
                # Лог раз в секунду (чтобы не спамить, так как цикл стал медленнее)
                print(f"[{self.state}] RSSI: {curr_rssi:.1f} ({trend_icon} {rssi_delta:.1f}) | Drops: {self.consecutive_drops}")

                # Логика "Мы едем не туда?" (Работает только в режиме сближения)
                if self.state == "APPROACH":
                    # Если сигнал упал более чем на 1.5 dBm по сравнению с прошлым пакетом
                    if rssi_delta < -1.5:
                        self.consecutive_drops += 1
                    elif rssi_delta > 0.5:
                        # Если сигнал вырос, сбрасываем счетчик ошибок
                        self.consecutive_drops = 0
                    
                    # Если 3 раза подряд сигнал падает -> КОРРЕКЦИЯ
                    if self.consecutive_drops >= 3:
                        print("🛑 ТРЕНД ОТРИЦАТЕЛЬНЫЙ! (Удаляемся). Корректировка...")
                        self.px.stop()
                        self.px.set_dir_servo_angle(30)   # Поворот колес
                        self.px.backward(SPEED_APPROACH)  # Чуть назад
                        time.sleep(0.8)
                        self.px.set_dir_servo_angle(-20)  # Меняем угол атаки
                        self.px.forward(SPEED_APPROACH)
                        time.sleep(0.5)
                        
                        self.consecutive_drops = 0        # Сброс
                        self.prev_avg_rssi = curr_rssi    # Сброс базы сравнения
                        continue

                # ---------------------------------------------------------
                # ШАГ 3: Обработка препятствий (Debounce - защита от глюков)
                if self.state not in ["FINISH", "VERIFY"]:
                    if dist is not None and dist > 0 and dist < OBSTACLE_DIST_CM:
                        self.obstacle_counter += 1
                    else:
                        self.obstacle_counter = 0 # Сброс, если путь чист
                    
                    if self.obstacle_counter >= 3:
                        print("⛔ СТЕНА (подтверждено 3 раза) -> AVOID")
                        self.state = "AVOID"
                        self.obstacle_counter = 0

                # ---------------------------------------------------------
                # ШАГ 4: Машина состояний
                
                if self.state == "AVOID":
                    # Логика объезда
                    direction = self.scan_surroundings()
                    self.px.backward(SPEED_SEARCH)
                    time.sleep(0.8)
                    self.px.set_dir_servo_angle(direction * 35)
                    self.px.forward(SPEED_SEARCH)
                    time.sleep(0.8)
                    self.state = "SEARCH"
                    # Сбрасываем "прошлое", так как мы сместились
                    self.prev_avg_rssi = curr_rssi 

                elif self.state == "SEARCH":
                    if curr_rssi > APPROACH_THRESHOLD:
                        print(f"✅ Захват луча ({curr_rssi:.1f}) -> APPROACH")
                        self.state = "APPROACH"
                        self.consecutive_drops = 0
                    else:
                        # Спираль
                        self.px.forward(SPEED_SEARCH)
                        self.px.set_dir_servo_angle(int(self.spiral_angle))
                        if self.spiral_angle < 0: self.spiral_angle += 1
                        else: self.spiral_angle = -35

                elif self.state == "APPROACH":
                    if curr_rssi >= TARGET_RSSI:
                        print("🧐 Сигнал пиковый. Остановка для проверки...")
                        self.state = "VERIFY"
                        self.px.stop()
                        self.verify_start_time = time.time()
                        self.verify_hits = 0
                        continue
                    
                    # Простое удержание курса (сложная логика теперь в блоке ТРЕНДА выше)
                    self.px.set_dir_servo_angle(0)
                    self.px.forward(SPEED_APPROACH)

                elif self.state == "VERIFY":
                    elapsed = time.time() - self.verify_start_time
                    
                    # Проверяем, держится ли сигнал
                    if curr_rssi >= TARGET_RSSI:
                        self.verify_hits += 1
                        print(f"   👍 Подтверждение {self.verify_hits}/{VERIFY_REQUIRED_HITS}")

                    if elapsed > VERIFY_DURATION:
                        if self.verify_hits >= VERIFY_REQUIRED_HITS:
                            print("🎉 ИСТОЧНИК НАЙДЕН! (Уверенность 100%)")
                            self.state = "FINISH"
                        else:
                            print("❌ Ложная тревога (сигнал скачет). Ищу дальше.")
                            self.px.backward(SPEED_APPROACH)
                            time.sleep(1.0)
                            self.state = "APPROACH"
                            self.prev_avg_rssi = curr_rssi

                elif self.state == "FINISH":
                    self.px.stop()
                    break

                # ---------------------------------------------------------
                # ШАГ 5: Обновление "прошлого" значения
                # Важно: обновляем его в конце цикла, чтобы в следующем сравнить с ним
                self.prev_avg_rssi = curr_rssi
                
                # Основная задержка уже встроена в get_averaged_rssi, 
                # поэтому тут sleep минимальный
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n⏹ Стоп.")
        finally:
            self.px.stop()
            self.center_head()

if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
