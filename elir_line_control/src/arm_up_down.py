#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState

class arm_up_down_mimic_control():

    def __init__(self):

        #Robot joints
        self.F_ARM_JOINTS = ['joint1_f','joint2_f']
        self.B_ARM_JOINTS = ['joint1_b','joint2_b']

        #Trajectory Duration
        self.trajectory_duration = rospy.rostime.Duration(0.8)
        

        #Creating our node,publisher and subscriber

        self.b_arm_subscriber = rospy.Subscriber('/joint1_b_controller/state', JointState, self.callback_joint1b) 
        self.f_arm_subscriber = rospy.Subscriber('/joint1_f_controller/state', JointState, self.callback_joint1f)  
        self.b_arm_subscriber = rospy.Subscriber('/joint2_b_controller/state', JointState, self.callback_joint2b) 
        self.f_arm_subscriber = rospy.Subscriber('/joint2_f_controller/state', JointState, self.callback_joint2f)  

        #self.b_arm_subscriber = rospy.Subscriber('elir/joint_states', JointState, self.callback_joints)

        self.key_subscriber = rospy.Subscriber('/key_vel', Twist, self.callback_joy)

        self.f_arm_publisher = rospy.Publisher('/elir/f_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.b_arm_publisher = rospy.Publisher('/elir/b_arm_trajectory_controller/command', JointTrajectory, queue_size=10) 

        rospy.spin()

    


    #Callback function implementing the pose value received
    # def callback_joints(self,data):
    #     self.current_joint1b = data.position[2]
    #     self.current_joint2b = data.position[3]
    #     self.current_joint1f = data.position[0]
    #     self.current_joint2f = data.position[1]

    def callback_joint1b(self,data):
        self.current_joint1b = data.current_pos

    def callback_joint2b(self,data):
        self.current_joint2b = data.current_pos

    def callback_joint1f(self,data):
        self.current_joint1f = data.current_pos

    def callback_joint2f(self,data):
        self.current_joint2f = data.current_pos     

    def callback_joy(self, data):
        key_f = 0.5*data.angular.z
        key_b = 0.5*data.angular.x

        
        self.UP_F_ARM_POINTS = [ -key_f + self.current_joint1f , key_f + self.current_joint2f]
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.F_ARM_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = self.trajectory_duration
        points.positions = self.UP_F_ARM_POINTS
        #Append the trajectory points and publish
        msg.points.append(points)
        self.f_arm_publisher.publish(msg)

        #B_ARM
        self.UP_B_ARM_POINTS = [ -key_b + self.current_joint1b , key_b + self.current_joint2b]
        #Get current time for header stamp
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.B_ARM_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = self.trajectory_duration
        points.positions = self.UP_B_ARM_POINTS
        #Append the trajectory points and publish
        msg.points.append(points)
        self.b_arm_publisher.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('elir_line_controller', anonymous=True)
        x = arm_up_down_mimic_control()
    except rospy.ROSInterruptException: pass