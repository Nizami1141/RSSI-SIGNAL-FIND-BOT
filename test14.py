import time
import math
from picarx import Picarx

# ================= 1. SETTINGS & THRESHOLDS =================
# --- PHYSICS CALIBRATION (CRITICAL) ---
# You must adjust these two numbers for your specific room!
RSSI_AT_1M = -50        # (A) What is the RSSI when you stand exactly 1m away?
PATH_LOSS_EXPONENT = 2.5 # (n) Environment factor (2.0 = empty field, 3.0-4.0 = crowded office)

# --- MISSION GOALS ---
# The robot will stop when it calculates it is between 0.5m and 0.9m
TARGET_DISTANCE_M = 0.7   # The ideal distance we want (Updated to 0.7m)
DIST_TOLERANCE = 0.2      # +/- 0.2 meters (So 0.5m to 0.9m is acceptable)

# --- MECHANICS ---
SPEED_FAST = 40
SPEED_SLOW = 20
MAX_TURN = 30
OBSTACLE_LIMIT = 25       # Centimeters

class KalmanFilter:
    """Smoothes out the jumpy Wi-Fi signal"""
    def __init__(self):
        self.R = 3.0  # High noise (measurements are messy)
        self.Q = 0.05 # Low process noise (robot moves smoothly)
        self.x = -60  # Initial guess
        self.P = 1.0

    def filter(self, measurement):
        if measurement is None: return self.x
        # Prediction
        p_pred = self.P + self.Q
        # Update
        K = p_pred / (p_pred + self.R)
        self.x = self.x + K * (measurement - self.x)
        self.P = (1 - K) * p_pred
        return self.x

class RobotBrain:
    def __init__(self):
        self.bot = Picarx()
        self.kf = KalmanFilter()
        self.last_valid_rssi = -100
        self.state = "SEARCH" # States: SEARCH, APPROACH, AVOID, FINISH

    def get_rssi(self):
        """Reads the raw Wi-Fi signal strength"""
        try:
            with open("/proc/net/wireless", "r") as f:
                lines = f.readlines()
                for line in lines:
                    if "wlan0:" in line:
                        parts = line.split()
                        # Usually the 4th item, remove dots
                        val = float(parts[3].replace('.', ''))
                        return val
        except:
            pass
        return None

    def calculate_distance(self, rssi):
        """
        CONVERTS SIGNAL (dBm) TO DISTANCE (Meters)
        Formula: Distance = 10 ^ ((A - RSSI) / (10 * n))
        """
        # Safety: If signal is weirdly high, assume we are very close
        if rssi > -10: return 0.1 
        
        # The Math
        power = (RSSI_AT_1M - rssi) / (10 * PATH_LOSS_EXPONENT)
        distance = 10 ** power
        return distance

    def scan_for_direction(self):
        """Stops and checks Left, Center, Right to find the strongest signal"""
        print("   👀 Looking for the best signal path...")
        self.bot.stop()
        best_rssi = -999
        best_angle = 0
        
        # Look Left (-30), Center (0), Right (30)
        for angle in [-30, 0, 30]:
            self.bot.set_dir_servo_angle(angle)
            time.sleep(0.5)
            
            # Take average of 3 readings
            readings = []
            for _ in range(3):
                r = self.get_rssi()
                if r: readings.append(r)
                time.sleep(0.1)
            
            if readings:
                avg = sum(readings) / len(readings)
                if avg > best_rssi:
                    best_rssi = avg
                    best_angle = angle
        
        print(f"   👉 Best direction found: {best_angle} degrees (RSSI: {best_rssi:.1f})")
        return best_angle

    def run(self):
        print(f"--- 🤖 ROBOT STARTED. Target: {TARGET_DISTANCE_M} meters (+/- {DIST_TOLERANCE}) ---")
        
        try:
            while True:
                # 1. READ SENSORS
                raw = self.get_rssi()
                obstacle_dist = self.bot.ultrasonic.read()
                
                if raw is not None:
                    rssi = self.kf.filter(raw)
                    self.last_valid_rssi = rssi
                else:
                    rssi = self.last_valid_rssi

                # 2. CALCULATE DISTANCE
                estimated_meters = self.calculate_distance(rssi)
                
                # Print Status
                print(f"[{self.state}] Signal: {rssi:.1f}dBm | Distance: {estimated_meters:.2f}m")

                # 3. SAFETY CHECK
                if obstacle_dist > 0 and obstacle_dist < OBSTACLE_LIMIT:
                    print("⛔ OBSTACLE DETECTED! Backing up...")
                    self.state = "AVOID"

                # 4. DECISION MAKING
                
                # --- CASE A: OBSTACLE ---
                if self.state == "AVOID":
                    self.bot.backward(SPEED_SLOW)
                    time.sleep(1.0)
                    self.bot.set_dir_servo_angle(MAX_TURN) # Turn right
                    self.bot.forward(SPEED_SLOW)
                    time.sleep(0.8)
                    self.state = "SEARCH"

                # --- CASE B: FINISHED (Target Range Reached) ---
                # This is the "Threshold" Logic
                elif (TARGET_DISTANCE_M - DIST_TOLERANCE) <= estimated_meters <= (TARGET_DISTANCE_M + DIST_TOLERANCE):
                    self.bot.stop()
                    self.bot.set_dir_servo_angle(0)
                    print(f"\n🎉 I FOUND YOU! Distance is {estimated_meters:.2f}m (Goal was {TARGET_DISTANCE_M}m)")
                    break

                # --- CASE C: SEARCH (Too far away) ---
                elif self.state == "SEARCH":
                    if estimated_meters < 6.0:
                        # If we are somewhat close (6m), switch to precision mode
                        self.state = "APPROACH"
                    else:
                        # If very far, move fast and spiral
                        self.bot.forward(SPEED_FAST)
                        # (Simple wiggle logic here or spiral logic)
                        self.bot.set_dir_servo_angle(0)

                # --- CASE D: APPROACH (Precision Mode) ---
                elif self.state == "APPROACH":
                    # Check if we are moving TOWARD or AWAY from the target
                    # We compare current calculated distance with previous calculated distance
                    # (Or we can just use RSSI trends)
                    
                    # Logic: If signal is weak, stop and look around
                    if estimated_meters > (TARGET_DISTANCE_M + 1.0):
                        # We are still drifting too far, scan again
                        angle = self.scan_for_direction()
                        self.bot.set_dir_servo_angle(angle)
                        self.bot.forward(SPEED_SLOW)
                        time.sleep(1.5) # Drive in that direction for 1.5 seconds
                    else:
                        # We are close, drive straight carefully
                        self.bot.set_dir_servo_angle(0)
                        self.bot.forward(SPEED_SLOW)

                time.sleep(0.1)

        except KeyboardInterrupt:
            print("\n🛑 Stopped by human.")
        finally:
            self.bot.stop()

if __name__ == "__main__":
    brain = RobotBrain()
    brain.run()
