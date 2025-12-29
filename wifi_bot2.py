import time
import statistics
from picarx import Picarx

# ================= КОНФИГУРАЦИЯ =================
TARGET_RSSI = -46         # Цель (dBm)
APPROACH_THRESHOLD = -55  # Порог сближения

# Настройки выборки (Burst)
BURST_SAMPLES = 15        # Для усреднения одного замера берем 15 микро-сигналов
SAMPLE_DELAY = 0.01

# --- НОВЫЕ НАСТРОЙКИ ПРОВЕРКИ (k из N) ---
VERIFY_TOTAL_CHECKS = 40  # Сколько всего раз проверить (N)
VERIFY_REQUIRED_HITS = 4  # Сколько раз нужно "пробить" порог, чтобы засчитать победу (k)

# Остальные настройки
OBSTACLE_DIST_CM = 25
SPEED_SEARCH = 40
SPEED_APPROACH = 30


# ================= КЛАСС WIFI =================

class WiFiSensor:
    def __init__(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def _read_raw(self):
        try:
            with open(self.filepath, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if self.interface + ":" in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            return float(parts[3].replace('.', ''))
        except: pass
        return None

    def get_averaged_rssi(self, count=15):
        """Берет пачку сигналов, чистит и усредняет."""
        readings = []
        for _ in range(count):
            val = self._read_raw()
            if val is not None: readings.append(val)
            time.sleep(SAMPLE_DELAY)
        
        if not readings: return None
        
        # Удаляем выбросы (шумы)
        if len(readings) >= 5:
            readings.sort()
            trim = int(len(readings) * 0.2)
            if trim > 0: readings = readings[trim:-trim]
            
        if not readings: return None
        return statistics.mean(readings)


# ================= КОНТРОЛЛЕР =================

class NavigationController:
    def __init__(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        
        self.state = "SEARCH"
        self.prev_avg_rssi = -100.0
        
        self.spiral_angle = -35
        self.consecutive_drops = 0
        self.obstacle_counter = 0
        
        # Переменные для новой логики проверки
        self.verify_counter = 0  # Текущий номер проверки (из 40)
        self.verify_hits = 0     # Количество успешных пробитий

        self.center_head()

    def center_head(self):
        try:
            self.px.set_cam_pan_angle(0)
            self.px.set_cam_tilt_angle(0)
        except: pass

    def scan_surroundings(self):
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

    def run(self):
        print(f"--- 🚀 ЗАПУСК (Logic: {VERIFY_REQUIRED_HITS} из {VERIFY_TOTAL_CHECKS}) ---")
        
        # Калибровка старта
        start_val = self.wifi.get_averaged_rssi(BURST_SAMPLES)
        if start_val: self.prev_avg_rssi = start_val

        try:
            while True:
                # 1. Чтение данных
                curr_rssi = self.wifi.get_averaged_rssi(BURST_SAMPLES)
                dist = self.px.ultrasonic.read()

                if curr_rssi is None: continue

                # 2. Логирование (чуть реже)
                rssi_delta = curr_rssi - self.prev_avg_rssi
                trend = "⬆️" if rssi_delta >= 0 else "⬇️"
                
                # Показываем лог движения, только если НЕ в режиме проверки
                if self.state != "VERIFY":
                    print(f"[{self.state}] RSSI: {curr_rssi:.1f} ({trend}) | Drops: {self.consecutive_drops}")

                # 3. Фильтр препятствий
                if self.state not in ["FINISH", "VERIFY"]:
                    if dist is not None and dist > 0 and dist < OBSTACLE_DIST_CM:
                        self.obstacle_counter += 1
                    else:
                        self.obstacle_counter = 0 
                    
                    if self.obstacle_counter >= 3:
                        print("⛔ СТЕНА -> AVOID")
                        self.state = "AVOID"
                        self.obstacle_counter = 0

                # 4. Машина состояний
                if self.state == "AVOID":
                    direction = self.scan_surroundings()
                    self.px.backward(SPEED_SEARCH)
                    time.sleep(0.8)
                    self.px.set_dir_servo_angle(direction * 35)
                    self.px.forward(SPEED_SEARCH)
                    time.sleep(0.8)
                    self.state = "SEARCH"
                    self.prev_avg_rssi = curr_rssi 

                elif self.state == "SEARCH":
                    if curr_rssi > APPROACH_THRESHOLD:
                        self.state = "APPROACH"
                    else:
                        self.px.forward(SPEED_SEARCH)
                        self.px.set_dir_servo_angle(int(self.spiral_angle))
                        if self.spiral_angle < 0: self.spiral_angle += 1
                        else: self.spiral_angle = -35

                elif self.state == "APPROACH":
                    # Логика тренда (если удаляемся - корректируем)
                    if rssi_delta < -1.5: self.consecutive_drops += 1
                    elif rssi_delta > 0.5: self.consecutive_drops = 0
                    
                    if self.consecutive_drops >= 3:
                        print("🛑 Удаляемся. Разворот...")
                        self.px.stop()
                        self.px.set_dir_servo_angle(35)
                        self.px.backward(SPEED_APPROACH)
                        time.sleep(1)
                        self.px.forward(SPEED_APPROACH)
                        self.consecutive_drops = 0
                        self.prev_avg_rssi = curr_rssi
                        continue

                    # Если сигнал пробил цель - начинаем ПРОВЕРКУ
                    if curr_rssi >= TARGET_RSSI:
                        print(f"🧐 Вход в зону цели ({curr_rssi}). Начинаю тест 40 замеров...")
                        self.state = "VERIFY"
                        self.px.stop()
                        self.px.set_dir_servo_angle(0)
                        
                        # СБРОС СЧЕТЧИКОВ ПЕРЕД ПРОВЕРКОЙ
                        self.verify_counter = 0
                        self.verify_hits = 0
                        continue
                    
                    self.px.set_dir_servo_angle(0)
                    self.px.forward(SPEED_APPROACH)

                elif self.state == "VERIFY":
                    # Увеличиваем счетчик попыток
                    self.verify_counter += 1
                    
                    # Проверяем текущий замер
                    if curr_rssi >= TARGET_RSSI:
                        self.verify_hits += 1
                        status = "✅ ПОПАЛ"
                    else:
                        status = "❌ МИМО"
                        
                    print(f"   [Тест {self.verify_counter}/{VERIFY_TOTAL_CHECKS}] {status} | Всего попаданий: {self.verify_hits}")

                    # Когда сделали все 40 замеров - подводим итог
                    if self.verify_counter >= VERIFY_TOTAL_CHECKS:
                        if self.verify_hits >= VERIFY_REQUIRED_HITS:
                            print(f"🎉 УСПЕХ! Сигнал подтвержден ({self.verify_hits} раз из {VERIFY_TOTAL_CHECKS}).")
                            self.state = "FINISH"
                        else:
                            print(f"🚫 ПРОВАЛ. Попаданий всего {self.verify_hits} (нужно {VERIFY_REQUIRED_HITS}). Ищу дальше.")
                            self.px.backward(SPEED_APPROACH)
                            time.sleep(1.0)
                            self.state = "APPROACH"
                            self.prev_avg_rssi = curr_rssi

                elif self.state == "FINISH":
                    self.px.stop()
                    break

                self.prev_avg_rssi = curr_rssi
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("\n⏹ Стоп.")
        finally:
            self.px.stop()

if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
