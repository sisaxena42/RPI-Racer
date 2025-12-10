import serial
import time
import serial.tools.list_ports

def connect_to_arduino():
    while True:
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if 'ttyACM' in port.device:
                try:
                    ser = serial.Serial(port.device, 19200, timeout=1.0)
                    time.sleep(5)  # Give time to reset
                    print(f"Connected to {port.device}")
                    return ser
                except serial.SerialException:
                    continue
        print("Waiting for Arduino...")
        time.sleep(1)

ser = connect_to_arduino()
ser.reset_input_buffer()

ser.write(b"RUN\n")
print("Sent RUN command to Arduino")

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
