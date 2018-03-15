import cv2
import numpy as np

data = np.load('map_data.npz')


original_map = cv2.cvtColor(np.array(data['arr_0']), cv2.COLOR_GRAY2BGR)
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
    cv2.line(map_color, pt1, pt2, (0,0,0), thickness=5, lineType=8, shift=0)

map_gray = cv2.cvtColor(map_color, cv2.COLOR_BGR2GRAY)

ret, binary_map = cv2.threshold(map_gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

opening_kernel = np.ones((3,3), np.uint8)
map_opening = cv2.morphologyEx(binary_map, cv2.MORPH_OPEN, opening_kernel, iterations = 2)

map_bg = cv2.dilate(map_opening, opening_kernel, iterations = 3)

map_dt = cv2.distanceTransform(map_opening, cv2.DIST_L2, 5)
ret, map_fg = cv2.threshold(map_dt, 0.2*map_dt.max(), 255, 0)

map_fg = np.uint8(map_fg)
#map_fg_invert = cv2.bitwise_not(map_fg)
map_unknown = cv2.subtract(map_bg, map_fg)

output = cv2.connectedComponentsWithStats(map_unknown)



num_labels = output[0]
labels = output[1]
stats = output[2]
centroids = output[3]

# print(num_labels)
# print(labels)
# print(stats)
# print(centroids)

for center in centroids:
    x = int(center[0])
    y = int(center[1])
    if(x < 12 or y < 22 or x > 200 or y > 240 ):
        pass
    else:
        cv2.line(original_map, (x,y), (x,y), (0,0,255), 5)

#markers = markers+1

#markers[map_unknown==255] = 0

#markers = cv2.watershed(map_color, markers)
#map_color[markers == -1] = [0,0,255]

# cv2.imshow("map_opening", map_opening)
# cv2.imshow("map_bg", map_bg)
# cv2.imshow("map_dt", map_dt)
# cv2.imshow("map_fg", map_fg)
# cv2.imshow("map_unknown", map_unknown)
#cv2.imshow("map_fg_invert", map_fg_invert)

#cv2.imshow("Sements", map_color)

cv2.imshow("Map with points of interest", original_map)
#cv2.imshow("Map Eroded", map_dilated)
#cv2.imshow("Map Edges", map_edges)
#cv2.imshow('Map Hough',map)
#cv2.imshow('Map', map_color)
#cv2.imshow("Segmented Map", map)
cv2.waitKey()