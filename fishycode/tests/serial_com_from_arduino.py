import serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1.0)
time.sleep(3)
ser.reset_input_buffer()
print('Serial Connection OK')

try:
    while True:
        time.sleep(0.5)
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8')
            print(line)
except KeyboardInterrupt:
    ser.close()