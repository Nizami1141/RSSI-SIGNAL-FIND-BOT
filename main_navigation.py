import sys
import time
from picarx import Picarx
from robot_hat import Ultrasonic

# Import custom modules
from vision_module import VisionSystem
from rssi_filter import RobustRSSIFilter

# --- Configuration ---
MODEL_PATH = "detect.tflite" # Path to your.tflite model file
RSSI_WINDOW = 20
RSSI_STABILITY = 5 # +/- 5dBm variance allowed
SAFE_DISTANCE = 20 # cm

def main():
    # 1. Initialize Hardware
    try:
        px = Picarx()
        # Initialize Ultrasonic with specific pins (D2, D3 common on Picar-X HAT)
        ultrasonic_sensor = Ultrasonic(Trig="D2", Echo="D3")
    except Exception as e:
        print(f"Hardware Initialization Failed: {e}")
        sys.exit(1)

    # 2. Initialize Software Subsystems
    rssi_filter = RobustRSSIFilter(RSSI_WINDOW, RSSI_STABILITY)
    vision_sys = VisionSystem(MODEL_PATH)
    vision_sys.start() # Start the vision thread
    
    print("Navigation System Online. Press Ctrl+C to stop.")
    
    try:
        while True:
            # --- INPUT STAGE ---
            
            # Read Sensors
            distance = ultrasonic_sensor.read()
            visual_obstacle = vision_sys.is_obstacle_detected()
            
            # Simulate RSSI input (In reality, replace with wifi reading function)
            # raw_rssi = get_wifi_strength()
            raw_rssi = -55 # Placeholder
            valid_rssi = rssi_filter.update(raw_rssi)
            
            # --- DECISION STAGE (Subsumption Architecture) ---
            
            # LAYER 1: Critical Safety (Ultrasonic)
            if distance > 0 and distance < SAFE_DISTANCE:
                print(f" Object at {distance}cm! Emergency Stop.")
                px.forward(0) # Stop
                time.sleep(0.1)
                # Reflexive Back-off
                px.set_dir_servo_angle(-30) # Turn wheels
                px.backward(50)
                time.sleep(0.5)
                px.forward(0)
                
            # LAYER 2: Visual Avoidance
            elif visual_obstacle:
                print(" Visual Obstacle Detected. Avoiding.")
                # Gentler avoidance maneuver
                px.set_dir_servo_angle(30) # Steer right
                px.forward(30) # Move slow
                
            # LAYER 3: Navigation (RSSI Homing)
            elif valid_rssi is not None:
                print(f"[NAV] Signal locked: {valid_rssi:.2f}dBm. Proceeding.")
                # Simple Logic: If signal strong, drive straight. 
                # (Complex homing would compare RSSI gradients)
                px.set_dir_servo_angle(0)
                px.forward(50)
                
            # LAYER 4: Search
            else:
                print(" Signal unstable. halting.")
                px.forward(0)
                # Optional: Spin to find signal stability?
            
            # Loop Pacing
            time.sleep(0.05) # 20Hz Control Loop

    except KeyboardInterrupt:
        print("\nShutdown Initiated.")
        px.stop()
        vision_sys.stop()
        vision_sys.join() # Wait for thread to close
        print("System Stopped.")

if __name__ == "__main__":
    main()
