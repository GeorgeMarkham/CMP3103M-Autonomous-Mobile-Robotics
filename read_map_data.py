import cv2
import numpy as np

data = np.load('map_data.npz')

map = np.array(data['arr_0'])

h, w = map.shape

erosion_kernel = np.ones((8,8), np.uint8)

map_dilated = cv2.dilate(map, erosion_kernel, iterations=1)

map_edges = cv2.Canny(map_dilated,100,200)

minLineLength = 1

lines = cv2.HoughLinesP(map_edges, rho=1,theta=np.pi/180, threshold=50,lines=np.array([]), minLineLength=minLineLength,maxLineGap=150)

map_color = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

for line in lines:
    pt1_x = line[0][0]
    pt1_y = line[0][1]
    pt2_x = line[0][2]
    pt2_y = line[0][3]

    pt1 = (pt1_x, pt1_y)
    pt2 = (pt2_x, pt2_y)
    if((pt1_x >= pt2_x - 10) and (pt1_x <= pt2_x + 10) and (pt1_y > pt2_y)):
        pt1 = (pt1_x, 0)
        pt2 = (pt2_x, h)
    if((pt1_y >= pt2_y - 10) and (pt1_y <= pt2_y + 10) and (pt1_x > pt2_x)):
        pt1 = (0, pt1_y)
        pt2 = (w, pt2_y)
    cv2.line(map_color, pt1, pt2, (0,255,0), thickness=5, lineType=8, shift=0)



#cv2.imshow("Map Original", map)
#cv2.imshow("Map Eroded", map_dilated)
cv2.imshow("Map Edges", map_edges)
#cv2.imshow('Map Hough',map)
cv2.imshow('Map', map_color)
cv2.waitKey()