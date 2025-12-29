import threading
import cv2
import numpy as np
import time
from tflite_runtime.interpreter import Interpreter

class VisionSystem(threading.Thread):
    def __init__(self, model_path, resolution=(320, 240)):
        super().__init__()
        self.model_path = model_path
        self.resolution = resolution
        self.running = True
        self.obstacle_detected = False
        self._lock = threading.Lock()
        
        # Load TFLite Model
        self.interpreter = Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        
        # Get Input Shape (Height, Width)
        self.input_shape = self.input_details['shape']
        self.h_input = self.input_shape[1]
        self.w_input = self.input_shape[2]

        # Initialize Camera
        # Using index 0. On some Pis with multiple cams, this might vary.
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, resolution)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, resolution[1])
        # Bufferless setting (backend dependent, generic approach below)

    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            
            # Pre-processing
            # Resize to model requirement (e.g., 300x300)
            frame_resized = cv2.resize(frame, (self.w_input, self.h_input))
            # Expand dims to create batch dimension 
            input_data = np.expand_dims(frame_resized, axis=0)
            
            # Run Inference
            self.interpreter.set_tensor(self.input_details['index'], input_data)
            self.interpreter.invoke()
            
            # Extract Results
            # Output indices depend on the specific model. 
            # For standard SSD MobileNet V2: 
            # 0: Locations, 1: Classes, 2: Scores, 3: Number of detections
            boxes = self.interpreter.get_tensor(self.output_details['index'])
            classes = self.interpreter.get_tensor(self.output_details[1]['index'])
            scores = self.interpreter.get_tensor(self.output_details[2]['index'])
            
            self._analyze_obstacles(boxes, scores)
            
    def _analyze_obstacles(self, boxes, scores, score_threshold=0.5):
        detected = False
        # Loop through detections
        for i in range(len(scores)):
            if scores[i] > score_threshold:
                # Box coordinates are normalized [ymin, xmin, ymax, xmax]
                ymin, xmin, ymax, xmax = boxes[i]
                
                # Heuristic: Obstacle is in the center lane and "close"
                # Center lane: x center between 0.3 and 0.7
                # Close: Height of box > 0.2 (20% of screen)
                box_center = (xmin + xmax) / 2
                box_height = ymax - ymin
                
                if 0.3 < box_center < 0.7 and box_height > 0.2:
                    detected = True
                    break # One obstacle is enough to trigger avoidance
        
        with self._lock:
            self.obstacle_detected = detected

    def is_obstacle_detected(self):
        with self._lock:
            return self.obstacle_detected

    def stop(self):
        self.running = False
        self.cap.release()
