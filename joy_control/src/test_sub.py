#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState, Joy

class joy_control():

    def __init__(self):

        self.joint1_b_subs = rospy.Subscriber('/joint1_b_controller/state', JointState, self.callback_joint1b) 
        self.joint1_f_subs = rospy.Subscriber('/joint1_f_controller/state', JointState, self.callback_joint1f) 
        
        self.joint2_b_subs = rospy.Subscriber('/joint2_b_controller/state', JointState, self.callback_joint2b) 
        self.joint2_f_subs = rospy.Subscriber('/joint2_f_controller/state', JointState, self.callback_joint2f) 

    def callback_joint1f(self,data):
        print("entrou")
        #self.current_joint1f = data.current_pos
        
    def callback_joint1b(self,data):
        print("entrou")
        #self.current_joint1b = data.current_pos

    def callback_joint2f(self,data):
        print("entrou")
        #self.current_joint2f = data.current_pos

    def callback_joint2b(self,data):
        print("entrou") 
        #self.current_joint2b = data.current_pos

if __name__ == '__main__':
    try:
        rospy.init_node('joy_controller', anonymous=True)
        x = joy_control()
    except rospy.ROSInterruptException: pass 