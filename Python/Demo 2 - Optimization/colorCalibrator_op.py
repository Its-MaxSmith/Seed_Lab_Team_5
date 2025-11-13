import cv2 as cv
import numpy as np

def nothing(x):
    pass

# Start camera
camera = cv.VideoCapture(0)
cv.namedWindow("HSV Tuner")

# Create trackbars for tuning HSV range
cv.createTrackbar("Lower H", "HSV Tuner", 0, 179, nothing)
cv.createTrackbar("Lower S", "HSV Tuner", 0, 255, nothing)
cv.createTrackbar("Lower V", "HSV Tuner", 0, 255, nothing)
cv.createTrackbar("Upper H", "HSV Tuner", 179, 179, nothing)
cv.createTrackbar("Upper S", "HSV Tuner", 255, 255, nothing)
cv.createTrackbar("Upper V", "HSV Tuner", 255, 255, nothing)

while True:
    ret, frame = camera.read()
    if not ret:
        break

    # Blur + convert to HSV
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    hsv_blur = cv.GaussianBlur(hsv, (5,5), 0)

    # Get trackbar positions
    l_h = cv.getTrackbarPos("Lower H", "HSV Tuner")
    l_s = cv.getTrackbarPos("Lower S", "HSV Tuner")
    l_v = cv.getTrackbarPos("Lower V", "HSV Tuner")
    u_h = cv.getTrackbarPos("Upper H", "HSV Tuner")
    u_s = cv.getTrackbarPos("Upper S", "HSV Tuner")
    u_v = cv.getTrackbarPos("Upper V", "HSV Tuner")

    lower_bound = np.array([l_h, l_s, l_v])
    upper_bound = np.array([u_h, u_s, u_v])

    # Create mask
    mask = cv.inRange(hsv_blur, lower_bound, upper_bound)
    result = cv.bitwise_and(frame, frame, mask=mask)

    # Show images
    cv.imshow("Original", frame)
    cv.imshow("Mask", mask)
    cv.imshow("Filtered", result)

    # Print center pixel HSV for reference
    center = hsv[hsv.shape[0]//2, hsv.shape[1]//2]
    print(f"Center HSV: H={center[0]}, S={center[1]}, V={center[2]}", end="\r")

    key = cv.waitKey(1) & 0xFF
    if key == 27:  # ESC key to exit
        break

camera.release()
cv.destroyAllWindows()
