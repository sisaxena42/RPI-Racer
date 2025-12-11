import serial
from hokuyolx import HokuyoLX
import time

### CONFIG ###
LIDAR_IP = "192.168.0.10"      # use whatever IP works for lidar_test.py
LIDAR_PORT = 10940
SERIAL_PORT = "/dev/ttyACM0"   # adjust if your Arduino shows up differently
BAUD = 19200
STOP_DISTANCE = 0.7            # meters
FRONT_ANGLE_DEG = 30           # +/- 30� window around straight ahead
SLEEP_SEC = 0.02               # loop delay
#############################

def main():
    # Connect to Arduino serial
    ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    time.sleep(2)  # allow Arduino to reset

    # Connect to LiDAR
    lidar = HokuyoLX(addr=(LIDAR_IP, LIDAR_PORT))

    print("Starting obstacle detection...")
    last_state = None

    while True:
        # Single scan
        ts, scan = lidar.get_dist()

        if scan is None or len(scan) == 0:
            continue

        n = len(scan)              # typically 1080 beams
        deg_per_beam = 270.0 / n   # UST-10LX FOV is 270�
        beams_each_side = int(FRONT_ANGLE_DEG / deg_per_beam)

        mid = n // 2
        start = max(0, mid - beams_each_side)
        end   = min(n, mid + beams_each_side)

        front_beams = scan[start:end]

        # convert mm -> m and filter valid
        valid = [d / 1000.0 for d in front_beams if 0 < d < 30000]

        if not valid:
            continue

        closest = min(valid)

        state = "STOP" if closest < STOP_DISTANCE else "GO"

        if state != last_state:
            print(f"{state}  (closest {closest:.2f} m)")
            ser.write((state + "\n").encode("ascii"))
            ser.flush()
            last_state = state

        time.sleep(SLEEP_SEC)

if __name__ == "__main__":
    main()
