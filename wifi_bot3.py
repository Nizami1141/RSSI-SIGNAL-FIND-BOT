import time
import statistics
import os
from picarx import Picarx

# ================= CONFIGURATION =================
# RSSI Goals
TARGET_RSSI = -45         # Target strength (approx 30-50cm from router)
APPROACH_THRESHOLD = -58  # Strength to switch from Search to Approach

# Sampling Settings
BURST_SAMPLES = 10        # Number of quick readings to average
SAMPLE_DELAY = 0.05       # Delay between readings

# Verification Logic (k out of N)
VERIFY_TOTAL_CHECKS = 30  # Total checks to perform once target is hit
VERIFY_REQUIRED_HITS = 5  # How many strong signals needed to confirm

# Movement Settings
OBSTACLE_DIST_CM = 25
SPEED_SEARCH = 40
SPEED_APPROACH = 25
SPEED_AVOID = 40

# ================= CLASS: WIFI SENSOR =================
class WiFiSensor:
    def __init__(self, interface="wlan0"):
        self.interface = interface
        self.filepath = "/proc/net/wireless"

    def _read_raw(self):
        """Reads a single RSSI value from the system."""
        try:
            # Check if file exists to avoid crashing
            if not os.path.exists(self.filepath):
                return None
                
            with open(self.filepath, "r") as f:
                lines = f.readlines()
                for line in lines:
                    if self.interface + ":" in line:
                        parts = line.split()
                        # Usually RSSI is the 4th column (index 3)
                        if len(parts) >= 4:
                            val_str = parts[3].replace('.', '')
                            return float(val_str)
        except Exception as e:
            # print(f"Error reading wifi: {e}")
            pass
        return None

    def get_averaged_rssi(self, count=10):
        """
        Takes multiple readings, removes outliers, returns mean.
        Essential for stable 'Finding Me' logic.
        """
        readings = []
        for _ in range(count):
            val = self._read_raw()
            if val is not None:
                readings.append(val)
            time.sleep(SAMPLE_DELAY)
        
        if not readings:
            return None
        
        # Simple noise filter: remove top 10% and bottom 10% outliers
        if len(readings) >= 5:
            readings.sort()
            trim_amt = int(len(readings) * 0.1)
            if trim_amt > 0:
                readings = readings[trim_amt:-trim_amt]
            
        if not readings: 
            return None
            
        return statistics.mean(readings)

# ================= CLASS: CONTROLLER =================
class NavigationController:
    def __init__(self):
        self.px = Picarx()
        self.wifi = WiFiSensor()
        
        self.state = "SEARCH"
        self.prev_avg_rssi = -100.0
        
        # Search variables
        self.spiral_angle = -30
        self.spiral_increment = 2
        
        # Approach variables
        self.consecutive_drops = 0
        
        # Obstacle variables
        self.obstacle_counter = 0

        # Verification variables
        self.verify_counter = 0
        self.verify_hits = 0

        self.center_head()
        self.px.stop()

    def center_head(self):
        self.px.set_cam_pan_angle(0)
        self.px.set_cam_tilt_angle(0)

    def check_obstacle(self):
        """Returns True if obstacle detected close by."""
        dist = self.px.ultrasonic.read()
        if dist and 0 < dist < OBSTACLE_DIST_CM:
            self.obstacle_counter += 1
        else:
            self.obstacle_counter = 0
            
        return self.obstacle_counter >= 2

    def avoid_obstacle(self):
        print("⚠️ OBSTACLE DETECTED -> Avoiding")
        self.px.stop()
        self.px.set_dir_servo_angle(0)
        self.px.backward(SPEED_AVOID)
        time.sleep(1)
        
        # Look left/right to decide turn
        self.px.set_cam_pan_angle(-30)
        time.sleep(0.5)
        dist_left = self.px.ultrasonic.read() or 0
        
        self.px.set_cam_pan_angle(30)
        time.sleep(0.5)
        dist_right = self.px.ultrasonic.read() or 0
        self.center_head()

        # Turn towards open space
        turn_angle = 35 if dist_left > dist_right else -35
        self.px.set_dir_servo_angle(turn_angle)
        self.px.forward(SPEED_AVOID)
        time.sleep(0.8)
        self.state = "SEARCH" # Reset to search after avoiding

    def run(self):
        print("=== 📡 WIFI TRACKER STARTED ===")
        print(f"Goal: {TARGET_RSSI} dBm")
        
        # Initial calibration
        start_val = self.wifi.get_averaged_rssi()
        if start_val: 
            self.prev_avg_rssi = start_val
            print(f"Initial Signal: {start_val:.1f} dBm")

        try:
            while True:
                # 1. Get Data
                curr_rssi = self.wifi.get_averaged_rssi(BURST_SAMPLES)
                if curr_rssi is None:
                    continue

                rssi_delta = curr_rssi - self.prev_avg_rssi
                
                # 2. Obstacle Check (Safety First)
                if self.state != "FINISH" and self.check_obstacle():
                    self.avoid_obstacle()
                    continue

                # 3. State Machine
                if self.state == "SEARCH":
                    self.handle_search(curr_rssi)
                
                elif self.state == "APPROACH":
                    self.handle_approach(curr_rssi, rssi_delta)

                elif self.state == "VERIFY":
                    self.handle_verify(curr_rssi)
                
                elif self.state == "FINISH":
                    self.px.stop()
                    print("🏆 TARGET REACHED. PROGRAM END.")
                    break

                # Update history
                self.prev_avg_rssi = curr_rssi
                # print(f"State: {self.state} | RSSI: {curr_rssi:.1f}")

        except KeyboardInterrupt:
            print("\n🛑 STOPPING ROBOT")
        finally:
            self.px.stop()

    # --- STATE HANDLERS ---

    def handle_search(self, curr_rssi):
        """
        Spirals outward to find a signal gradient.
        """
        if curr_rssi > APPROACH_THRESHOLD:
            print(f"🔎 Signal found ({curr_rssi}). Switching to APPROACH.")
            self.state = "APPROACH"
            self.consecutive_drops = 0
            return

        # Spiral logic
        print(f"SEARCHING... Signal: {curr_rssi:.1f}")
        self.px.forward(SPEED_SEARCH)
        self.px.set_dir_servo_angle(self.spiral_angle)
        
        # Slowly straighten the wheels to spiral out
        if self.spiral_angle < 0:
            self.spiral_angle += 0.5  # Slowly increase radius
        else:
            self.spiral_angle = -35   # Reset if too wide

    def handle_approach(self, curr_rssi, delta):
        """
        'Finding Me' Logic:
        If signal gets stronger (+delta), drive straight.
        If signal gets weaker (-delta), adjust steering.
        """
        if curr_rssi >= TARGET_RSSI:
            print(f"🎯 Threshold hit ({curr_rssi}). Verifying...")
            self.state = "VERIFY"
            self.verify_counter = 0
            self.verify_hits = 0
            self.px.stop()
            return

        # Signal Logic
        if delta >= 0:
            # Signal improving or stable -> Go Straight
            self.consecutive_drops = 0
            self.px.set_dir_servo_angle(0)
            self.px.forward(SPEED_APPROACH)
            print(f"Element Found (Up): {curr_rssi:.1f} ⬆️")
        else:
            # Signal dropping -> Correct course
            self.consecutive_drops += 1
            print(f"Signal Drop: {curr_rssi:.1f} ⬇️ ({self.consecutive_drops})")

            if self.consecutive_drops >= 3:
                # We lost the trail. Reverse and turn.
                print("⚠️ Lost signal trail. Re-orienting.")
                self.px.stop()
                self.px.backward(SPEED_APPROACH)
                self.px.set_dir_servo_angle(30) # Turn wheels
                time.sleep(0.8)
                self.consecutive_drops = 0
            else:
                # Minor correction (Wiggle)
                self.px.set_dir_servo_angle(15) 
                self.px.forward(SPEED_APPROACH)

    def handle_verify(self, curr_rssi):
        """
        Stops and takes N samples. Needs k hits to pass.
        """
        self.verify_counter += 1
        
        hit = "❌"
        if curr_rssi >= TARGET_RSSI:
            self.verify_hits += 1
            hit = "✅"
            
        print(f"   [Verify {self.verify_counter}/{VERIFY_TOTAL_CHECKS}] {curr_rssi} {hit}")

        if self.verify_counter >= VERIFY_TOTAL_CHECKS:
            if self.verify_hits >= VERIFY_REQUIRED_HITS:
                print(f"🎉 CONFIRMED! ({self.verify_hits}/{VERIFY_TOTAL_CHECKS} hits)")
                self.state = "FINISH"
            else:
                print(f"🚫 FALSE ALARM. Only {self.verify_hits} hits. Returning to SEARCH.")
                # CRITICAL FIX: Go back to SEARCH, not APPROACH.
                # If we failed here, the approach angle was probably wrong.
                # Spiraling allows us to find a new vector.
                self.px.backward(SPEED_APPROACH)
                time.sleep(1.0)
                self.state = "SEARCH"
                self.spiral_angle = -35 # Reset spiral

if __name__ == "__main__":
    bot = NavigationController()
    bot.run()
