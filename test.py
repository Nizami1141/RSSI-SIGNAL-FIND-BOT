import time
import subprocess
import re
from picarx import Picarx

# --- НАСТРОЙКИ ---
INTERFACE = "wlan0"    
TARGET_RSSI = -35      
OBSTACLE_DIST = 30     

px = Picarx()

def get_wifi_rssi_linux(interface="wlan0"):
    try:
        cmd = f"iwconfig {interface}"
        result = subprocess.check_output(cmd, shell=True).decode()
        match = re.search(r"level=(-\d+)", result)
        if match:
            return int(match.group(1))
        else:
            return None
    except Exception:
        return None

def avoid_obstacle():
    print("--- ПРЕПЯТСТВИЕ! ---")
    px.stop()
    time.sleep(0.2)
    px.backward(20)
    time.sleep(0.5)
    
    # ИСПРАВЛЕНО: Поворот колес
    px.set_dir_servo_angle(35) 
    px.forward(20)
    time.sleep(1)
    px.set_dir_servo_angle(0)
    px.stop()

def main():
    print(f"--- ЗАПУСК (PicarX v2.0) ---")
    print("Робот ищет Wi-Fi...")
    
    # --- ИСПРАВЛЕННЫЕ КОМАНДЫ ДЛЯ КАМЕРЫ ---
    px.set_cam_pan_angle(0)   # Камера смотрит прямо (было servo1)
    px.set_cam_tilt_angle(0)  # Камера смотрит в горизонт (было servo2)
    px.set_dir_servo_angle(0) # Колеса прямо

    last_rssi = get_wifi_rssi_linux(INTERFACE)
    if last_rssi is None: last_rssi = -100
        
    print(f"Стартовый сигнал: {last_rssi} dBm")
    
    try:
        while True:
            # 1. Проверка стен
            distance = px.ultrasonic.read()
            if distance > 0 and distance < OBSTACLE_DIST:
                avoid_obstacle()
                continue

            # 2. Проверка Wi-Fi
            current_rssi = get_wifi_rssi_linux(INTERFACE)
            if current_rssi is None: continue
            
            diff = current_rssi - last_rssi
            print(f"Signal: {current_rssi} dBm | Diff: {diff}")

            if current_rssi >= TARGET_RSSI:
                print("!!! ЦЕЛЬ НАЙДЕНА !!!")
                px.stop()
                break
            
            # Логика движения
            if diff > 0: 
                # Горячо
                px.set_dir_servo_angle(0)
                px.forward(30)
            elif diff < 0:
                # Холодно
                px.stop()
                px.set_dir_servo_angle(30)
                px.forward(30)
                time.sleep(0.5)
            else:
                px.forward(20)
            
            last_rssi = current_rssi
            time.sleep(0.5)

    except KeyboardInterrupt:
        px.stop()
        print("\nОстановка.")

if __name__ == "__main__":
    main()
