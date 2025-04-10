import serial
import time

# Adjust the port and baud rate
ser = serial.Serial('/dev/ttyACM0', 19200, timeout=1.0)
time.sleep(2)  # Wait for Arduino to reset

print("Connected to Arduino")

# Send trigger
ser.write(b"RUN\n")
print("Sent RUN command to Arduino")

# Read the response
try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode("utf-8").strip()
            print(f"[Arduino] {line}")
            if line == "Done":
                break
except KeyboardInterrupt:
    pass
finally:
    ser.close()
    print("Serial connection closed")
