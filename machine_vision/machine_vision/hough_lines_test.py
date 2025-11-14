import cv2
import math
import numpy as np

img=cv2.imread('Intersection.png')
height, width, channels = img.shape
x_lower = int(width/4)
x_upper = int(3*(width/4))
y_lower = int(2*(height/5))
y_upper = int(height)
cropped_img = img[y_lower:y_upper, x_lower:x_upper]
cv2.imwrite('CroppedIntersection.png', cropped_img)

# Apply binary mask to extract tape lines
hsv = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2HSV)
lower_color_bound = np.array([15,80,80])
upper_color_bound = np.array([40,255,255])
mask = cv2.inRange(hsv, lower_color_bound, upper_color_bound)
masked_img = cv2.bitwise_and(cropped_img, cropped_img, mask=mask)
print(masked_img)
cv2.imwrite('BinaryIntersection.png', masked_img)

# Apply Hough Line Transform
grayscale_img = cv2.cvtColor(masked_img, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(grayscale_img, 30, 150, apertureSize=3)
lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=50, minLineLength=20, maxLineGap=20)
print(lines)
angle_list = []
for points in lines:
    x1, y1, x2, y2 = points[0]
    angle = 0-math.atan((y2-y1)/(x2-x1))
    angle_list.append(angle)
    print(math.degrees(angle))
    cv2.line(masked_img, (x1, y1), (x2, y2), (255, 0, 255), 2)
cv2.imwrite('IntersectionHoughLines.jpg', masked_img)





