import collections

class RobustRSSIFilter:
    def __init__(self, window_size=20, stability_threshold=3):
        # Using deque for efficient FIFO queue
        self.window = collections.deque(maxlen=window_size)
        self.threshold = stability_threshold
        self.last_valid_rssi = None

    def update(self, rssi_val):
        """
        Adds a new sample and returns the validated signal strength.
        """
        self.window.append(rssi_val)
        
        # Requirement: Check for 3 consecutive stable values
        if len(self.window) >= 3:
            # Convert to list for indexing
            data = list(self.window)
            
            # Iterate backwards (newest to oldest) to minimize latency.
            # We look for a triplet (i, i-1, i-2)
            for i in range(len(data) - 1, 1, -1):
                val1 = data[i]
                val2 = data[i-1]
                val3 = data[i-2]
                
                # Check deviations
                diff1 = abs(val1 - val2)
                diff2 = abs(val2 - val3)
                
                if diff1 <= self.threshold and diff2 <= self.threshold:
                    # Stable triplet found.
                    # Average them to smooth quantization noise
                    avg_rssi = (val1 + val2 + val3) / 3.0
                    self.last_valid_rssi = avg_rssi
                    return avg_rssi
                    
        # If we fall through, no stable sequence exists in the current window
        return self.last_valid_rssi
