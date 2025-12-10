import cv2
import numpy as np
import serial
import time

# === Serial Setup ===
try:
    arduino = serial.Serial('/dev/ttyACM0', 19200, timeout=1)
    time.sleep(2)  # Give time for Arduino to reset
    print("Serial connected to Arduino.")
except serial.SerialException:
    print("Error: Could not connect to Arduino.")
    exit()

# === HSV Bounds (You may need to tweak these for your fish) ===
lower_hsv = np.array([0,170 , 25])
upper_hsv = np.array([179, 255, 255])

# === Camera Setup ===
camera = cv2.VideoCapture(0)
FRAME_WIDTH = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
FRAME_HEIGHT = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
camera.set(3, 640)
camera.set(4, 480)

if not camera.isOpened():
    print("Error: Camera not detected")
    exit()

print(f"Camera opened with resolution {FRAME_WIDTH}x{FRAME_HEIGHT}")

# === Main Loop ===
while True:
    ret, frame = camera.read()
    if not ret:
        print("Error: Failed to grab frame")
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        center_x = x + w // 2
        center_y = y + h // 2

        # Draw debug overlays
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        # Send to Arduino
        data = f"{center_x},{center_y}\n"
        try:
            arduino.write(data.encode())
            print(f"Sent to Arduino: {data.strip()}")
        except serial.SerialException:
            print("Error: Failed to write to Arduino")
    else:
        print("Fish not detected.")

    # Show debug windows (optional during demo)
    cv2.imshow("Tracking", frame)
    cv2.imshow("Mask", mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# === Cleanup ===
camera.release()
cv2.destroyAllWindows()
arduino.close()
print("Program ended.")
