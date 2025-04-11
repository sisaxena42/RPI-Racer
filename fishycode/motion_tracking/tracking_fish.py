import cv2
import numpy as np
import serial
import time

arduino = serial.Serial('/dev/ttyAMA0',9600)
time.sleep(2)

lower_hsv = np.array([77,163,12])
upper_hsv = np.array([94,255,100])

camera = cv2.VideoCapture(0)
if not camera.isOpened():
	print("Error: Camera not detected")
	exit()

while True:
	ret, frame = camera.read()
	if not ret:
		print("Error: Failed to grab frame")
		break

	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	mask = cv2.inRange(hsv,lower_hsv,upper_hsv)
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

	if contours:
		largest_contour = max(contours, key=cv2.contourArea)
		x,y,w,h = cv2.boundingRect(largest_contour)
		center_x = x + w // 2
		center_y = y + h // 2
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
	    cv2.circle(frame,(center_x,center_y),5,(0,0,255),-1)
		position_data = f"X={center_x},Y={center_y}\n"
		arduino.write(position_data.encode())
		print(f"Sent position: X={center_x},Y={center_y}")

	cv2.imshow('Tracking',frame)
	cv2.imshow("Mask", mask)
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

camera.release()
cv2.destroyAllWindows()
ser.close()
