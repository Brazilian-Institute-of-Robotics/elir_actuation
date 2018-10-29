#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class line_mimic_control():

    def __init__(self):
        
        #Creating our node,publisher and subscriber
        
        self.key_subscriber = rospy.Subscriber('/key_vel', Twist, self.callback) 
        self.f_arm_publisher = rospy.Publisher('/joint1_f_controller/command', Float64, queue_size=10)
        self.b_arm_publisher = rospy.Publisher('/joint1_b_controller/command', Float64, queue_size=10)
        self.b_arm_position = rospy.Subscriber('/joint1_b_controller/state', JointState, queue_size=10)
        #self.f_arm_joints = ['joint1_f']
        #self.b_arm_joints = ['joint1_b', 'joint2_b']
        #self.trajectory_duration = rospy.rostime.Duration(0.8)

        

        rospy.spin()
    #Callback function implementing the pose value received
    def callback(self, data):    
        joint_states = JointState()
        joint_states.position = self.b_arm_position.position 
        key_vel = joint_state.state
        offset = 0.1*data.angular.z
        self.f_arm_publisher.publish((key_vel + offset)) 
        
        #self.f_arm_publisher.publish(key_vel)

if __name__ == '__main__':
    try:
        rospy.init_node('elir_line_controller', anonymous=True)
        x = line_mimic_control()
    except rospy.ROSInterruptException: pass