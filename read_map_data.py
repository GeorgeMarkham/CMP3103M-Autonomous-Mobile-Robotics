import cv2
import numpy as np

data = np.load('map_data.npz')

map = np.array(data['arr_0'])

erosion_kernel = np.ones((11,11), np.uint8)

map_dilated = cv2.dilate(map, erosion_kernel, iterations=1)

map_edges = cv2.Canny(map_dilated,100,200)

minLineLength = 1

lines = cv2.HoughLinesP(map_edges, rho=1,theta=np.pi/180, threshold=100,lines=np.array([]), minLineLength=minLineLength,maxLineGap=10)

map_color = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

for line in lines:
    pt1 = (line[0][0], line[0][1])
    pt2 = (line[0][2], line[0][3])
    cv2.line(map_color, pt1, pt2, (0,255,0), thickness=5, lineType=8, shift=0)

#cv2.imshow("Map Original", map)
#cv2.imshow("Map Eroded", map_dilated)
#cv2.imshow("Map Edges", map_edges)
#cv2.imshow('Map Hough',map)
cv2.imshow('Map', map_color)
cv2.waitKey()