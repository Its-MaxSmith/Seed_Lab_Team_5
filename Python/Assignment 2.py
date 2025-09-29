from time import sleep
import numpy as np
import cv2


fileName = input("File Name: ")
	
# initialize the camera. If channel 0 doesn't work, try channel 1
camera = cv2.VideoCapture(0)
	
# Let the camera warmup
sleep(1)
	
while True:
    ret, image = camera.read()
    if not ret:
        print("Could not capture image from camera!")
        break

    # Show the live feed
    cv2.imshow("Live Feed", image)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):  # save on 's'
        print("Saving image " + fileName)
        cv2.imwrite(fileName, image)
	
# Release the camera
camera.release()
image = cv2.imread(fileName)
imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
kernal = np.ones((5, 5), np.uint8)
lower = np.array([35, 0, 0])
upper = np.array([85, 255, 255])
mask = cv2.inRange(imageHSV, lower, upper)
result = cv2.bitwise_and(image, image, mask=mask)
erosion = cv2.erode(mask, kernal, iterations=1)
dilation = cv2.dilate(mask, kernal, iterations=1)
opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernal)
closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernal)
gradient = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, kernal)
tophat = cv2.morphologyEx(mask, cv2.MORPH_TOPHAT, kernal)
blackhat = cv2.morphologyEx(mask, cv2.MORPH_BLACKHAT, kernal)
cv2.imshow("Original", image)
cv2.imshow("Mask", mask)
cv2.imshow("Result", result)
cv2.imshow("Erosion", erosion)
cv2.imshow("Dilation", dilation)
cv2.imshow("Opening", opening)
cv2.imshow("Closing", closing)
cv2.imshow("Gradient", gradient)
cv2.imshow("Tophat", tophat)
cv2.imshow("Blackhat", blackhat)
cv2.waitKey(0)
cv2.destroyAllWindows()