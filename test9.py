import time
from collections import deque
from picarx import Picarx

# --- ROBOT INITIALIZATION ---
px = Picarx()

# --- WI-FI SEARCH SETTINGS ---
TARGET_RSSI = -40      # Ideal target for cost function
STOP_THRESHOLD = -46   # Stop when signal >= -46 dBm
WINDOW_SIZE = 5        # Reduced for faster response
rssi_history = deque(maxlen=WINDOW_SIZE)

# --- OBSTACLE AVOIDANCE ---
OBSTACLE_LIMIT = 20    # cm - stop before obstacle

# --- SEARCH STATE ---
class SearchState:
    FORWARD = "forward"
    SCAN_LEFT = "scan_left"
    SCAN_RIGHT = "scan_right"
    EXPLORE = "explore"

state = SearchState.FORWARD
scan_angle = 0
scan_samples = []
no_improvement_count = 0
best_rssi = -100

# ================= FUNCTIONS =================

def get_rssi_linux():
    """Read Wi-Fi signal strength from Linux"""
    try:
        with open("/proc/net/wireless", "r") as f:
            lines = f.readlines()
            for line in lines:
                if "wlan0" in line:
                    parts = line.split()
                    rssi = float(parts[3].replace('.', ''))
                    return rssi
    except Exception as e:
        print(f"RSSI Error: {e}")
        return None

def stop_robot():
    """Stop all robot movement"""
    px.forward(0)
    px.set_dir_servo_angle(0)

def avoid_obstacle():
    """Back up and turn away from obstacle"""
    print("⛔ OBSTACLE DETECTED - AVOIDING")
    px.stop()
    time.sleep(0.2)
    
    # Back up
    px.set_dir_servo_angle(0)
    px.backward(35)
    time.sleep(1.2)
    px.stop()
    
    # Turn away
    px.set_dir_servo_angle(-40)
    px.forward(30)
    time.sleep(1.5)
    px.stop()
    
    # Clear history
    rssi_history.clear()
    return -100  # Reset best RSSI

def scan_for_best_direction():
    """Scan left and right to find strongest signal direction"""
    print("🔍 SCANNING for best direction...")
    scan_results = []
    
    # Scan angles: -40, -20, 0, 20, 40
    angles = [-40, -20, 0, 20, 40]
    
    for angle in angles:
        px.set_dir_servo_angle(angle)
        px.forward(25)  # Move slowly while scanning
        time.sleep(0.5)
        
        # Take measurement
        rssi = get_rssi_linux()
        if rssi:
            scan_results.append((angle, rssi))
            print(f"  Angle {angle}°: {rssi:.1f} dBm")
        
        time.sleep(0.3)
    
    px.stop()
    
    # Find best direction
    if scan_results:
        best_angle, best_rssi = max(scan_results, key=lambda x: x[1])
        print(f"✨ Best direction: {best_angle}° ({best_rssi:.1f} dBm)")
        return best_angle, best_rssi
    
    return 0, -100

def move_forward_cautiously(speed=35):
    """Move forward while checking for obstacles"""
    px.set_dir_servo_angle(0)
    px.forward(speed)

# ================= MAIN LOOP =================

print(f"--- PicarX: Wi-Fi Seeker (Target: {STOP_THRESHOLD} dBm) ---")
time.sleep(2)

try:
    while True:
        # --- 1. CHECK SENSORS ---
        distance = px.ultrasonic.read()
        rssi = get_rssi_linux()
        
        # --- 2. OBSTACLE AVOIDANCE (PRIORITY #1) ---
        if distance > 0 and distance < OBSTACLE_LIMIT:
            best_rssi = avoid_obstacle()
            no_improvement_count = 0
            state = SearchState.EXPLORE
            continue
        
        # --- 3. WI-FI SEARCH LOGIC ---
        if rssi is None:
            print("❌ Wi-Fi adapter error")
            px.stop()
            time.sleep(0.5)
            continue
        
        rssi_history.append(rssi)
        
        # Wait for enough samples
        if len(rssi_history) < WINDOW_SIZE:
            print(f"📊 Collecting data... {len(rssi_history)}/{WINDOW_SIZE}")
            move_forward_cautiously(25)
            time.sleep(0.2)
            continue
        
        avg_rssi = sum(rssi_history) / len(rssi_history)
        print(f"📡 Signal: {avg_rssi:.1f} dBm | State: {state}")
        
        # --- CHECK IF TARGET REACHED ---
        if avg_rssi >= STOP_THRESHOLD:
            print("\n" + "="*50)
            print(f"🎉 TARGET FOUND! Signal: {avg_rssi:.1f} dBm")
            print("="*50 + "\n")
            stop_robot()
            break
        
        # --- ADAPTIVE SEARCH STRATEGY ---
        
        # Update best signal seen
        if avg_rssi > best_rssi:
            best_rssi = avg_rssi
            no_improvement_count = 0
        else:
            no_improvement_count += 1
        
        # STATE: FORWARD - Moving toward signal
        if state == SearchState.FORWARD:
            if avg_rssi > best_rssi - 3:  # Signal is good or improving
                print("✅ SIGNAL STRONG - Moving forward")
                move_forward_cautiously(40)
                no_improvement_count = 0
            else:
                # Signal degrading - need to search
                state = SearchState.EXPLORE
                print("⚠️ Signal weakening - Switching to EXPLORE")
        
        # STATE: EXPLORE - Lost the signal, find it again
        elif state == SearchState.EXPLORE:
            best_angle, scan_rssi = scan_for_best_direction()
            
            if scan_rssi > avg_rssi:
                # Found better direction
                print(f"↗️ Better signal found - Turning {best_angle}°")
                px.set_dir_servo_angle(best_angle)
                px.forward(35)
                time.sleep(1.0)
                
                best_rssi = scan_rssi
                rssi_history.clear()
                state = SearchState.FORWARD
                no_improvement_count = 0
            else:
                # Still lost - try rotating more
                print("🔄 No improvement - Rotating to search")
                px.set_dir_servo_angle(45)
                px.forward(30)
                time.sleep(1.5)
                rssi_history.clear()
        
        # Emergency: Stuck without improvement for too long
        if no_improvement_count > 15:
            print("⚠️ STUCK - Performing exploration maneuver")
            px.set_dir_servo_angle(-45)
            px.forward(35)
            time.sleep(2.0)
            px.stop()
            
            rssi_history.clear()
            no_improvement_count = 0
            best_rssi = -100
            state = SearchState.EXPLORE
        
        time.sleep(0.15)

except KeyboardInterrupt:
    print("\n🛑 Stopped by user")
    stop_robot()
except Exception as e:
    print(f"\n❌ Error: {e}")
    stop_robot()
finally:
    stop_robot()
    print("Program ended")
