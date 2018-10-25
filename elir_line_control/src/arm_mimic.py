#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist

class line_mimic_control():

    def __init__(self):
        #Creating our node,publisher and subscriber
        self.key_subscriber = rospy.Subscriber('/key_vel', Twist, self.callback)
        self.f_arm_publisher = rospy.Publisher('/f_arm_trajectory_controller/command', Float64, queue_size=10)
        self.b_arm_publisher = rospy.Publisher('/b_arm_trajectory_controller/command', Float64, queue_size=10)

        rospy.spin()
    #Callback function implementing the pose value received
    def callback(self, data):
        key_vel = 0.3*data.angular.z
        self.f_arm_publisher.publish(key_vel)
        self.f_arm_publisher.publish(key_vel)

if __name__ == '__main__':
    try:
        rospy.init_node('elir_line_controller', anonymous=True)
        x = line_mimic_control()
    except rospy.ROSInterruptException: pass