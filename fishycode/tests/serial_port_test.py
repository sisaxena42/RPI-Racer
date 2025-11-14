import serial, time, serial.tools.list_ports

def connect():
    while True:
        for p in serial.tools.list_ports.comports():
            if 'Arduino' in p.description or 'ttyACM' in p.device:
                try:
                    s = serial.Serial(p.device, 19200, timeout=1)
                    time.sleep(2)
                    print(f"Connected on {p.device}")
                    return s
                except serial.SerialException:
                    pass
        print("Waiting for Arduino...")
        time.sleep(1)

ser = connect()
ser.write(b"RUN\n")

while True:
    try:
        if ser.in_waiting:
            print(ser.readline().decode(errors='ignore').strip())
    except OSError:
        print("Lost connection, reconnecting...")
        ser.close()
        ser = connect()
