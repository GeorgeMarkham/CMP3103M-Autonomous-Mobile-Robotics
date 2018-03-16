import math
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

import tf.transformations

class assignment_1:
    def __init__(self):
        rate = rospy.Rate(1)

        self.move_client = actionlib.SimpleActionClient('/turtlebot/move_base/', MoveBaseAction)
        self.move_client.wait_for_server()
        self.turn_count = 0
        self.point_counter = 0

        self.blue_found = False
        self.green_Found = False
        self.red_found = False
        self.yellow_found = False

        self.moving = False
        #WINDOW TO SHOW WHAT THE ROBOT SEES#
        cv2.namedWindow("Map", 1)
        #cv2.namedWindow("Robot view", 1)
        self.bridge = CvBridge()
        
        cv2.startWindowThread()

        #SET PUBLISHERS#
        self.vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=10)

        #SET SUBSCRIBERS#
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.img_callback)
        self.map_sub = rospy.Subscriber("/turtlebot/move_base/global_costmap/costmap", OccupancyGrid, self.map_callback)
        self.laser_scan_sub = rospy.Subscriber("/turtlebot/scan", LaserScan, self.laser_scan_callback)
        self.odom_sub = rospy.Subscriber("/turtlebot/odom", Odometry, self.odom_callback)

    def img_callback(self, img_data):
        img = self.bridge.imgmsg_to_cv2(img_data, 'bgr8')
        #cv2.imshow("Robot view", img)
        
        blue = [np.array([15, 0, 0]), np.array([250, 0, 0])]
        green = [np.array([ 0,  15,  0]), np.array([0, 250, 0])]
        red = [np.array([0, 0, 15]), np.array([0, 0, 250])]
        yellow = [np.array([0, 172, 172]), np.array([0, 206, 206])]

        h,w,_ = img.shape
        
        M_blue = cv2.moments(cv2.inRange(img, blue[0], blue[1]))
        M_green = cv2.moments(cv2.inRange(img, green[0], green[1]))
        M_red = cv2.moments(cv2.inRange(img, red[0], red[1]))
        M_yellow = cv2.moments(cv2.inRange(img, yellow[0], yellow[1]))

        #print(M)
        if M_blue['m00'] > 0 and not self.blue_found:
            self.move_client.cancel_goal()
            self.moving = True
            print("Found blue!")
            print(M_red['m00'])

            if M_blue['m00'] >= 5000000.0:
                self.moving = False
                self.blue_found = True

            cx = int(M_blue['m10']/M_blue['m00'])
            cy = int(M_blue['m01']/M_blue['m00'])

            print cx

            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
        elif M_green['m00'] > 0 and not self.green_found:
            self.move_client.cancel_goal()
            self.moving = True
            print("Found green!")
            print(M_green['m00'])

            if M_green['m00'] >= 5000000.0:
                self.moving = False
                self.green_found = True

            cx = int(M_green['m10']/M_green['m00'])
            cy = int(M_green['m01']/M_green['m00'])

            print cx

            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
        elif M_red['m00'] > 0 and not self.red_found:
            self.move_client.cancel_goal()
            self.moving = True
            print("Found Red!")
            print(M_red['m00'])

            if M_red['m00'] >= 5000000.0:
                self.moving = False
                self.red_found = True

            cx = int(M_red['m10']/M_red['m00'])
            cy = int(M_red['m01']/M_red['m00'])

            print cx

            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
        elif M_yellow['m00'] > 0 and not self.yellow_found:
            self.move_client.cancel_goal()
            self.moving = True
            print("Found yellow!")
            print(M_yellow['m00'])

            if M_yellow['m00'] >= 5000000.0:
                self.moving = False
                self.yellow_found = True

            cx = int(M_yellow['m10']/M_yellow['m00'])
            cy = int(M_yellow['m01']/M_yellow['m00'])

            print cx

            err = cx - w/2
            twist = Twist()
            twist.linear.x = 0.2
            twist.angular.z = -float(err) / 100
            self.vel_pub.publish(twist)
        elif self.turn_count == 0:
            self.move_client.cancel_goal()
            twist = Twist()
            twist.angular.z = 0.5
            self.vel_pub.publish(twist)
            self.turn_count += 1
            self.moving = False
        elif (self.move_client.get_state == actionlib.SimpleGoalState.ACTIVE or self.move_client.get_state == actionlib.CommState.ACTIVE) and self.move_client.get_result():
            self.point_counter += 1
            print(self.point_counter)
        elif self.move_client.get_state() == actionlib.TerminalState.ABORTED:
            self.moving = False
            #cv2.imshow("Robot view", img)
        elif not self.moving:
            try:
                point = self.area_centers[self.point_counter]
                print(point)
                #result = self.navigate_to_point(point)

                move_to = MoveBaseGoal()
                move_to.target_pose.header.frame_id = "map"
                move_to.target_pose.header.stamp = rospy.Time.now()
                move_to.target_pose.pose.position.x = point[0]
                move_to.target_pose.pose.position.y = point[1]
                move_to.target_pose.pose.orientation.w = 1.0
                self.move_client.send_goal(move_to)
                self.moving = True
                #res = self.move_client.wait_for_result()
                #if res:
                    # self.moving = False
                    # self.point_counter += 1
                    # print("turning...")
                    # twist = Twist()
                    # twist.angular.z = 0.5
                    # self.vel_pub.publish(twist)
                    # self.turn_count += 1
                    # self.moving = False
            except rospy.ROSInterruptException:
                rospy.loginfo("Navigation test finished.")
            #     if result:
            #         print("Done:", point)
            #         self.point_counter += 1
            # except rospy.ROSInterruptException:
            #     rospy.loginfo("Navigation test finished.")

        #for point in area_centers:
        #    try:
        #        result = self.navigate_to_point(point)
        #        if result:
        #            print("Done:", point)
        #    except rospy.ROSInterruptException:
        #        rospy.loginfo("Navigation test finished.")
        #else:
        #    pass

        self.current_frame = img


    def map_callback(self, map_data):
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y

        map_w = map_data.info.width
        map_h = map_data.info.height
        resolution = map_data.info.resolution



        map = np.array(map_data.data).reshape(map_h, map_w)

        offset = (origin_x, origin_y)


        length = len(map_data.data)
        map = np.zeros((map_h,map_w), dtype = "uint8")
        

        for i in range(1,map_h):
            for j in range(1,map_w):
                map[i-1, j-1] = 255-int(float(map_data.data[(i-1)*map_w+j])/100*255)
        map = cv2.flip(map, 0)
        ret,binary_map = cv2.threshold(map,127,255,cv2.THRESH_BINARY)

        map_centers = self.find_map_centers(binary_map)


        color_map = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

        self.area_centers = []

        for center in map_centers:
            cv2.line(color_map, center, center, (0,0,255), 3)
            self.area_centers.append(self.get_world_pt((center[1], center[0]), offset, resolution))

        #print(self.area_centers)

        cv2.imshow("Map", color_map)
        
    def odom_callback(self, odom_data):
        #print(odom_data.pose)
        pass

    def laser_scan_callback(self, laser_data):
        #print laser_data
        scan_angle_min = laser_data.angle_min
        scan_angle_max = laser_data.angle_max

        laser_ranges = laser_data.ranges
        ranges_length = len(laser_ranges)

        should_turn = False
        turn_direction = False # False = Left

        for i in range(ranges_length):
            if(laser_ranges[i] <= 1.3):
                move = Twist()
                move.angular.z = -1.0
                self.vel_pub.publish(move)
            

    def find_map_centers(self, map):
        original_map = map
        map = map

        h, w = map.shape

        erosion_kernel = np.ones((8,8), np.uint8)

        map_dilated = cv2.dilate(map, erosion_kernel, iterations=1)

        map_edges = cv2.Canny(map_dilated,100,200)

        minLineLength = 1

        lines = cv2.HoughLinesP(map_edges, rho=1,theta=np.pi/180, threshold=50,lines=np.array([]), minLineLength=minLineLength,maxLineGap=150)

        map_color = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)


        for line in lines[0]:
            pt1_x = line[0]
            pt1_y = line[1]
            pt2_x = line[2]
            pt2_y = line[3]

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

        ret, map_fg = cv2.threshold(map_opening, 0.2*map_opening.max(), 255, 0)

        map_fg = np.uint8(map_fg)
        map_unknown = cv2.subtract(map_bg, map_fg)

        contours, hierarchy = cv2.findContours(map_unknown.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        center_pts = []
        centers = []

        for contour in contours:
            #peri = cv2.arcLength(contour, True)
            #approx = cv2.approxPolyDP(contour, 0.02 * peri, True)

            if len(contour) >= 4:
                contour_moment = cv2.moments(contour)
                center_pt = (int(contour_moment["m10"]/contour_moment["m00"]), int(contour_moment["m01"]/contour_moment["m00"]))
                centers.append(center_pt)
                #print(contour)

        for center in centers:
            x = int(center[0])
            y = int(center[1])
            if(x < 12 or y < 22 or x > 200 or y > 240 or original_map[y][x] == 0):
                pass
            else:
                center_pts.append(center)
        
        return center_pts
    def get_world_pt(self, point, offset, resolution):

        #(0,0)
        x_map = point[1]
        y_map = point[0]

        x_world = (x_map * resolution) + offset[0]
        y_world = (y_map * resolution) + offset[1]

        world_pt = (x_world, y_world)

        return world_pt

rospy.init_node('assignment_1')
iv = assignment_1()
rospy.spin()


#LASER SCAN OBSTACLE AVOIDANCE#

# scan_angle_min = laser_data.angle_min
# scan_angle_max = laser_data.angle_max

# laser_ranges = laser_data.ranges
# ranges_length = len(laser_ranges)

# should_turn = False
# turn_direction = False # False = Left

# for i in range(ranges_length):
#     if(laser_ranges[i] <= 1.7 and i <= (ranges_length/2)):
#         should_turn = True
#         turn_direction = True
#     elif(laser_ranges[i] <= 1.7 and i <= (ranges_length/2)):
#         should_turn = True
#         pass

# if(should_turn and turn_direction):
#     move_msg = Twist()
#     move_msg.linear.x = 0.0
#     move_msg.angular.z = -0.5

#     self.vel_pub.publish(move_msg)
# elif(should_turn and turn_direction == False):
#     move_msg = Twist()
#     move_msg.linear.x = 0.0
#     move_msg.angular.z = 0.5

#     self.vel_pub.publish(move_msg)
# else:
#     move_msg = Twist()
#     move_msg.linear.x = 0.1
#     move_msg.angular.z = 0.0
#     self.vel_pub.publish(move_msg)


#The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.
