import serial
import time

# Adjust the serial port if needed (use /dev/ttyUSB0 or /dev/ttyACM1 if not ACM0)
SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 19200

try:
    print(f"Connecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Allow time for Arduino reset

    print("Sending RUN command...")
    ser.write(b"RUN\n")

    # Wait for "Done" message from Arduino
    while True:
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode("utf-8").strip()
                print(f"[Arduino] {line}")
                if line == "Done":
                    break
        except OSError as e:
            print(f"[Serial Error] {e}")
            break

except serial.SerialException as e:
    print(f"[Connection Error] {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed.")
