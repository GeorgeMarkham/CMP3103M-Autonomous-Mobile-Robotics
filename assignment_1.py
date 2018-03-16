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
        self.time = rospy.Time()
        #print(self.time)
        #WINDOW TO SHOW WHAT THE ROBOT SEES#
        cv2.namedWindow("Map", 1)
        cv2.namedWindow("Robot view", 1)
        self.bridge = CvBridge()
        
        cv2.startWindowThread()

        #SET PUBLISHERS#
        self.vel_pub = rospy.Publisher('/turtlebot/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/turtlebot/move_base_simple/goal', PoseStamped, queue_size=10)

        #SET SUBSCRIBERS#
        self.image_sub = rospy.Subscriber("/turtlebot/camera/rgb/image_raw", Image, self.img_callback)
        self.map_sub = rospy.Subscriber("/turtlebot/move_base/global_costmap/costmap", OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber("/turtlebot/odom", Odometry, self.odom_callback)

    def img_callback(self, img_data):
        img = self.bridge.imgmsg_to_cv2(img_data, 'bgr8')

        cv2.imshow("Robot view", img)

        self.current_frame = img

    def map_callback(self, map_data):
        #np.savetxt('map_data', map_data.data)
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y
        #origin_z = map_data.info.origin.position.z

        map_w = map_data.info.width
        map_h = map_data.info.height
        resolution = map_data.info.resolution

        #print(map_data.info.origin)


        map = np.array(map_data.data).reshape(map_h, map_w)

        offset = (origin_x, origin_y)

        #np.savetxt('map_data', map)

        length = len(map_data.data)
        map = np.zeros((map_h,map_w), dtype = "uint8")
        

        for i in range(1,map_h):
            for j in range(1,map_w):
                map[i-1, j-1] = 255-int(float(map_data.data[(i-1)*map_w+j])/100*255)
        map = cv2.flip(map, 0)
        ret,binary_map = cv2.threshold(map,127,255,cv2.THRESH_BINARY)
        #binary_map = cv2.flip(binary_map, 0)
        #print(binary_map)
        #cv2.imshow("Map", binary_map)

        map_centers = self.find_map_centers(binary_map)


        color_map = cv2.cvtColor(map, cv2.COLOR_GRAY2BGR)

        area_centers = []

        for center in map_centers:
            cv2.line(color_map, center, center, (0,0,255), 3)
            #print(center[0], center[1])
            #print(self.get_world_pt((center[1], center[0]), (-5.62,-6.2), resolution))
            area_centers.append(self.get_world_pt((center[1], center[0]), offset, resolution))

        print(area_centers)
        #print(self.get_world_pt((0,0), (-5.62,-6.2), resolution))
        #print(self.get_world_pt((map_h/2,map_w/2),offset,resolution))
        #print(resolution)

        cv2.imshow("Map", color_map)


        # now = rospy.get_rostime()


        # pose = PoseStamped()
        # pose.header.stamp = rospy.Time.now()
        # pose.header.frame_id = "map"
        # pose.pose.position.x = 0.0
        # pose.pose.position.y = 0.0
        # pose.pose.position.z = 0.0

        # quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        # pose.pose.orientation.x = quaternion[0]
        # pose.pose.orientation.y = quaternion[1]
        # pose.pose.orientation.z = quaternion[2]
        # pose.pose.orientation.w = quaternion[3]
        # print(pose)
        # self.goal_pub.publish(pose)

        #self.goal_pub.publish(goal)
        for point in area_centers:
            try:
                result = self.navigate_to_point(point)
                if result:
                    print("Done:", point)
                    print("Looking for a color...")
                    img = self.current_frame
                    lower = np.array([ 0,  15,  0])
                    upper = np.array([0, 250, 0])
                    mask = cv2.inRange(img, lower, upper)

                    h,w,_ = img.shape

                    M = cv2.moments(mask)

                    print(M)
                    counter = 0
                    if M['m00'] > 0:
                        print("Found a color!")
                        cx = int(M['m10']/M['m00'])
                        cy = int(M['m01']/M['m00'])

                        print cx

                        err = cx - w/2
                        twist = Twist()
                        twist.linear.x = 0.2
                        twist.angular.z = -float(err) / 100
                        self.vel_pub.publish(twist)
                    else:
                        if counter == 0:
                            twist = Twist()
                            twist.angular.z = 0.5
                            self.vel_pub.publish(twist)
                            counter += 1
            except rospy.ROSInterruptException:
                rospy.loginfo("Navigation test finished.")
        
    def navigate_to_point(self, point):
        self.currently_moving = True
        move_client = actionlib.SimpleActionClient('/turtlebot/move_base/', MoveBaseAction)
        move_client.wait_for_server()

        move_to = MoveBaseGoal()
        move_to.target_pose.header.frame_id = "map"
        move_to.target_pose.header.stamp = rospy.Time.now()
        move_to.target_pose.pose.position.x = point[0]
        move_to.target_pose.pose.position.y = point[1]
        move_to.target_pose.pose.orientation.w = 1.0

        move_client.send_goal(move_to)
        wait_result = move_client.wait_for_result()
        if(move_client.get_state() == actionlib.TerminalState.ABORTED):
            print("Plan aborted :(")
            return move_client.get_state()

        print(move_client.get_state())

        if not wait_result:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return move_client.get_result()
        
    def odom_callback(self, odom_data):
        #print(odom_data.pose)
        pass

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
