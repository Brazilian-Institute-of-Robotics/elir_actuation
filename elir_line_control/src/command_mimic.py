#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist

class line_mimic_control():

    def __init__(self):
        #Creating our node,publisher and subscriber
        self.key_subscriber = rospy.Subscriber('/key_vel', Twist, self.callback)
        self.traction_f_publisher = rospy.Publisher('/traction_f_controller/command', Float64, queue_size=10)
        self.traction_b_publisher = rospy.Publisher('/traction_b_controller/command', Float64, queue_size=10)
        self.traction_ap_publisher = rospy.Publisher('/traction_ap_controller/command', Float64, queue_size=10)

        rospy.spin()
    #Callback function implementing the pose value received
    def callback(self, data):
        key_vel = 3*data.angular.z
        self.traction_f_publisher.publish(key_vel)
        self.traction_b_publisher.publish(key_vel)
        self.traction_ap_publisher.publish(-key_vel)

if __name__ == '__main__':
    try:
        rospy.init_node('elir_line_controller', anonymous=True)
        x = line_mimic_control()
    except rospy.ROSInterruptException: pass