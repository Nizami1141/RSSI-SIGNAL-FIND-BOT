import time
import statistics
import os
import random
import subprocess 
from picarx import Picarx

# ================= НАСТРОЙКИ =================
# RSSI (Сила сигнала)
TARGET_RSSI = -48.5       # Цель (Финиш)
APPROACH_THRESHOLD = -53  # Переход к сближению

# ТАЙМИНГИ ПОИСКА (8 сек прямо, 3 сек влево)
TIME_STRAIGHT = 8.0   
TIME_TURN = 3.0       

# Настройки WiFi
BURST_SAMPLES = 5         
SAMPLE_DELAY = 0.05       

# Проверка (Verify)
VERIFY_TOTAL_CHECKS = 20  
VERIFY_REQUIRED_HITS = 4  

# Скорости
OBSTACLE_DIST_CM = 25
SPEED_SEARCH = 40     
SPEED_APPROACH = 25
SPEED_AVOID = 40

# Интервал повтора аудио при поиске (сек)
SCAN_AUDIO_INTERVAL = 8.0  

# ================= CLASS: WIFI SENSOR =================
class WiFiSensor:
    def __init__(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def _read_raw(self):
        try:
            if not os.path.exists(self.filepath):
                return None
            with open(self.filepath, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if self.interface + ":" in line:
                        parts = line.split()
                        if len(parts) >= 4:
                            val_str = parts[3].replace('.', '')
                            return float(val_str)
        except Exception:
            pass
        return None

    def get_averaged_rssi(self, count=5):
        readings = []
        for _ in range(count):
            val = self._read_raw()
            if val is not None:
                readings.append(val)
            time.sleep(SAMPLE_DELAY)
        if not readings: return None
        if len(readings) >= 5:
            readings.sort()
            trim_amt = int(len(readings) * 0.1)
            if trim_amt > 0:
                readings = readings[trim_amt:-trim_amt]
        if not readings: return None
        return statistics.mean(readings)

# ================= CLASS: CONTROLLER =================
class NavigationController:
    def __init__(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        
        # --- ОБНОВЛЕННЫЕ ИМЕНА ФАЙЛОВ ---
        # Звуки поиска (когда ищет)
        self.scan_sounds = ["audio1.wav", "audio2.wav"]
        
        # Звуки победы (когда нашел)
        self.found_sounds = ["audio3.wav", "audio4.wav", "audio5.wav"]
        
        # Хак таймера: ставим время в прошлое, чтобы первый звук был МГНОВЕННО при старте
        self.last_audio_time = time.time() - SCAN_AUDIO_INTERVAL - 1
        
        # State
        self.state = "SEARCH"
        self.prev_avg_rssi = -100.0
        self.search_start_time = time.time()
        
        self.consecutive_drops = 0
        self.obstacle_counter = 0
        self.verify_counter = 0
        self.verify_hits = 0

        self.center_head()
        self.px.stop()
        
        # --- МАКСИМАЛЬНАЯ ГРОМКОСТЬ ПРИ ЗАПУСКЕ ---
        print("🔊 Setting MAX volume...")
        os.system("amixer set PCM 100% > /dev/null 2>&1")
        os.system("amixer set Headphone 100% > /dev/null 2>&1")
        os.system("amixer set Master 100% > /dev/null 2>&1")

    def _play_sound_background(self, filename):
        """Играет звук в фоне (не тормозит робота)"""
        if os.path.exists(filename):
            subprocess.Popen(["aplay", "-q", filename])
        else:
            print(f"⚠️ Audio file missing: {filename}")

    def play_scan_sound(self):
        current_time = time.time()
        # Играем, только если прошел интервал (8 сек)
        if current_time - self.last_audio_time > SCAN_AUDIO_INTERVAL:
            if self.state != "VERIFY" and self.scan_sounds:
                sound_file = random.choice(self.scan_sounds)
                self._play_sound_background(sound_file)
                self.last_audio_time = current_time

    def play_victory_sound(self):
        self.px.stop()
        if self.found_sounds:
            sound_file = random.choice(self.found_sounds)
            print(f"🔊 VICTORY SOUND: {sound_file}")
            # Еще раз форсируем громкость
            os.system("amixer set PCM 100% > /dev/null 2>&1")
            os.system(f"aplay -q {sound_file} &")

    def center_head(self):
        self.px.set_cam_pan_angle(0)
        self.px.set_cam_tilt_angle(0)

    def check_obstacle(self):
        dist = self.px.ultrasonic.read()
        if dist and 0 < dist < OBSTACLE_DIST_CM:
            self.obstacle_counter += 1
        else:
            self.obstacle_counter = 0
        return self.obstacle_counter >= 2

    def avoid_obstacle(self):
        print("⚠️ OBSTACLE -> Backing up & Turning LEFT")
        self.px.stop()
        self.px.set_dir_servo_angle(0)
        self.px.backward(SPEED_AVOID)
        time.sleep(1.0)
        
        self.px.set_dir_servo_angle(-35) 
        self.px.forward(SPEED_AVOID)
        time.sleep(0.8)
        
        self.state = "SEARCH"
        self.search_start_time = time.time() 

    def run(self):
        print("=== 📡 WIFI TRACKER (Using audio1-5.wav) ===")
        
        start_val = self.wifi.get_averaged_rssi()
        if start_val: 
            self.prev_avg_rssi = start_val
        self.search_start_time = time.time()

        # Играем звук поиска СРАЗУ при запуске
        self.play_scan_sound()

        try:
            while True:
                # 1. Проверка таймера звука
                self.play_scan_sound()

                # 2. Данные WiFi
                curr_rssi = self.wifi.get_averaged_rssi(BURST_SAMPLES)
                if curr_rssi is None: continue
                rssi_delta = curr_rssi - self.prev_avg_rssi
                
                # 3. Препятствия
                if self.state != "FINISH" and self.check_obstacle():
                    self.avoid_obstacle()
                    continue

                # 4. Логика движения
                if self.state == "SEARCH":
                    self.handle_search(curr_rssi)
                elif self.state == "APPROACH":
                    self.handle_approach(curr_rssi, rssi_delta)
                elif self.state == "VERIFY":
                    self.handle_verify(curr_rssi)
                elif self.state == "FINISH":
                    self.px.stop()
                    print(f"🏆 FOUND! RSSI: {curr_rssi}")
                    self.play_victory_sound()
                    time.sleep(5) # Ждем пока договорит
                    break

                self.prev_avg_rssi = curr_rssi

        except KeyboardInterrupt:
            print("\n🛑 STOP")
        finally:
            self.px.stop()

    def handle_search(self, curr_rssi):
        if curr_rssi > APPROACH_THRESHOLD:
            print(f"🔎 CAUGHT ({curr_rssi}). To APPROACH.")
            self.state = "APPROACH"
            self.consecutive_drops = 0
            return

        now = time.time()
        elapsed = now - self.search_start_time
        cycle_duration = TIME_STRAIGHT + TIME_TURN
        phase_time = elapsed % cycle_duration
        
        if phase_time < TIME_STRAIGHT:
            # Прямо
            # print(f"SEARCH [Straight]: {phase_time:.1f}s | {curr_rssi:.1f}")
            self.px.set_dir_servo_angle(0)
            self.px.forward(SPEED_SEARCH)
        else:
            # Налево
            # print(f"SEARCH [Left]: {phase_time:.1f}s | {curr_rssi:.1f}")
            self.px.set_dir_servo_angle(-35) 
            self.px.forward(SPEED_SEARCH)

    def handle_approach(self, curr_rssi, delta):
        if curr_rssi >= TARGET_RSSI:
            print(f"🎯 STRONG ({curr_rssi}). Verifying...")
            self.state = "VERIFY"
            self.verify_counter = 0
            self.verify_hits = 0
            self.px.stop()
            return

        if delta >= 0:
            self.consecutive_drops = 0
            self.px.set_dir_servo_angle(0)
            self.px.forward(SPEED_APPROACH)
            print(f"APPROACH: Good ({curr_rssi:.1f}) ⬆️")
        else:
            self.consecutive_drops += 1
            print(f"APPROACH: Weak ({curr_rssi:.1f}) ⬇️ {self.consecutive_drops}")

            if self.consecutive_drops >= 3:
                print("⚠️ Lost. Back & Left.")
                self.px.stop()
                self.px.backward(SPEED_APPROACH)
                self.px.set_dir_servo_angle(-30) 
                time.sleep(0.8)
                self.consecutive_drops = 0
            else:
                self.px.set_dir_servo_angle(-20) 
                self.px.forward(SPEED_APPROACH)

    def handle_verify(self, curr_rssi):
        self.verify_counter += 1
        hit = "✅" if curr_rssi >= TARGET_RSSI else "❌"
        if hit == "✅": self.verify_hits += 1
            
        print(f"   [Verify {self.verify_counter}/{VERIFY_TOTAL_CHECKS}] {curr_rssi} {hit}")

        if self.verify_counter >= VERIFY_TOTAL_CHECKS:
            if self.verify_hits >= VERIFY_REQUIRED_HITS:
                self.state = "FINISH"
            else:
                print(f"🚫 False. Back & Left.")
                self.px.backward(SPEED_APPROACH)
                self.px.set_dir_servo_angle(-30) 
                time.sleep(1.0)
                self.state = "SEARCH"
                self.search_start_time = time.time()

if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
