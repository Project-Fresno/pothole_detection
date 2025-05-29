import cv2
import numpy as np


def nothing(x):
    pass


# Load an image from file
image_path = "image.png"  # change this to your actual image path
image = cv2.imread(image_path)
if image is None:
    raise ValueError("Failed to load image. Check the path!")

cv2.namedWindow("HSV Mask Tuner", cv2.WINDOW_NORMAL)
cv2.resizeWindow("HSV Mask Tuner", 800, 400)

# Create trackbars
cv2.createTrackbar("H Min", "HSV Mask Tuner", 0, 179, nothing)
cv2.createTrackbar("H Max", "HSV Mask Tuner", 179, 179, nothing)
cv2.createTrackbar("S Min", "HSV Mask Tuner", 0, 255, nothing)
cv2.createTrackbar("S Max", "HSV Mask Tuner", 30, 255, nothing)
cv2.createTrackbar("V Min", "HSV Mask Tuner", 200, 255, nothing)
cv2.createTrackbar("V Max", "HSV Mask Tuner", 255, 255, nothing)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

while True:
    h_min = cv2.getTrackbarPos("H Min", "HSV Mask Tuner")
    h_max = cv2.getTrackbarPos("H Max", "HSV Mask Tuner")
    s_min = cv2.getTrackbarPos("S Min", "HSV Mask Tuner")
    s_max = cv2.getTrackbarPos("S Max", "HSV Mask Tuner")
    v_min = cv2.getTrackbarPos("V Min", "HSV Mask Tuner")
    v_max = cv2.getTrackbarPos("V Max", "HSV Mask Tuner")

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])

    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(image, image, mask=mask)

    combined = np.hstack((image, result))
    cv2.imshow("HSV Mask Tuner", combined)

    key = cv2.waitKey(1)
    if key == 27:  # ESC to quit
        break

print(f"{','.join(map(str, lower))},{','.join(map(str, upper))}")
# print(f"HSV (low): {tuple(lower)}")
# print(f"HSV (high): {tuple(upper)}")

cv2.destroyAllWindows()
