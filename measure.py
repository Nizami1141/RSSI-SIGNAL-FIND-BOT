# save as rssi_test.py
import time
from subprocess import check_output, DEVNULL
import re

INTERFACE = "wlan0"

def get_rssi():
    out = check_output(f"iw dev {INTERFACE} link", shell=True, stderr=DEVNULL).decode(errors="ignore")
    m = re.search(r"signal:\s*(-?\d+\.?\d*)\s*dBm", out)
    if m:
        return float(m.group(1))
    return -100.0

print("Нажми Ctrl+C для выхода\n")
try:
    while True:
        rssi = get_rssi()
        print(f"RSSI: {rssi:.1f} dBm")
        time.sleep(0.5)
except KeyboardInterrupt:
    pass
