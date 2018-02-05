import rospy
from geometry_msgs.msg import Twist

rospy.init_node('mover_pub')

publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

rate = rospy.Rate(1)

while not rospy.is_shutdown():

    cmd = Twist()

    cmd.angular.z = 0.5
    publisher.publish(cmd)
    
    print cmd

    rate.sleep()

rospy.spin()
