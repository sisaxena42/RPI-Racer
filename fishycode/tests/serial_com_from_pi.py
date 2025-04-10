import serial as serial
import time

ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1.0)
time.sleep(3)
ser.reset_input_buffer()
print('Serial Connection OK')

try:
    while True:
        time.sleep(1)
        print("sending message to arduino")
        line = ser.write("From Pi\n".encode('utf-8'))
        #print(line)
except KeyboardInterrupt:
    print("connection severed")
    ser.close()