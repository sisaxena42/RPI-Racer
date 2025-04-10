import cv2
import numpy as np

def nothing(x):
	pass

camera = cv2.VideoCapture(0)
camera.set(3,640)
camera.set(4,480)

cv2.namedWindow("Trackbars")
cv2.createTrackbar("LH","Trackbars",0,179,nothing)
cv2.createTrackbar("LS","Trackbars",0,255,nothing)
cv2.createTrackbar("LV","Trackbars",0,255,nothing)
cv2.createTrackbar("UH","Trackbars",179,179,nothing)
cv2.createTrackbar("US","Trackbars",255,255,nothing)
cv2.createTrackbar("UV","Trackbars",255,255,nothing)

while True:
	ret,frame = camera.read()
	if not ret:
		print("Failed to grab frame")
		break
	hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
	
	lh = cv2.getTrackbarPos("LH","Trackbars")
	ls = cv2.getTrackbarPos("LS","Trackbars")
	lv = cv2.getTrackbarPos("LV","Trackbars")
	uh = cv2.getTrackbarPos("UH","Trackbars")
	us = cv2.getTrackbarPos("US","Trackbars")
	uv = cv2.getTrackbarPos("UV","Trackbars")

	lower_bound = np.array([lh,ls,lv])
	upper_bound = np.array([uh, us, uv])

	mask = cv2.inRange(hsv, lower_bound, upper_bound)
	result = cv2.bitwise_and(frame, frame, mask=mask)
	cv2.imshow("Original",frame)
	cv2.imshow("Mask",mask)
	cv2.imshow("Filtered",result)

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

camera.release()
cv2.destroyAllWindows()


