#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from dynamixel_msgs.msg import Pose

class turtlebot():

    def __init__(self):
        #Creating our node,publisher and subscriber
        rospy.init_node('elir_line_controller', anonymous=True)
        self.key_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    #Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

if __name__ == '__main__':
    try:
        #Testing our function
        x = turtlebot()
        x.move2goal()

except rospy.ROSInterruptException: pass