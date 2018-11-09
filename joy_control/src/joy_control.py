#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState, Joy

class joy_control():

    def __init__(self):
        #Creating our node,publisher and subscriber
        self.b_arm_subscriber = rospy.Subscriber('elir/joint_states', JointState, self.callback)

        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.joint1f_publisher = rospy.Publisher('/joint1_f_controller/command', Float64, queue_size=10)
        self.joint1b_publisher = rospy.Publisher('/joint1_b_controller/command', Float64, queue_size=10)
        self.joint2f_publisher = rospy.Publisher('/joint2_f_controller/command', Float64, queue_size=10)
        self.joint2b_publisher = rospy.Publisher('/joint2_b_controller/command', Float64, queue_size=10)  

        self.traction_f_publisher = rospy.Publisher('/traction_f_controller/command', Float64, queue_size=10)
        self.traction_b_publisher = rospy.Publisher('/traction_b_controller/command', Float64, queue_size=10)
        self.traction_ap_publisher = rospy.Publisher('/traction_ap_controller/command', Float64, queue_size=10)

        self.joint1_b_subs = rospy.Subscriber('/joint1_b_controller/state', JointState, self.callback_joint1b) 
        self.joint1_f_subs = rospy.Subscriber('/joint1_f_controller/state', JointState, self.callback_joint1f) 
        
        self.joint2_b_subs = rospy.Subscriber('/joint2_b_controller/state', JointState, self.callback_joint2b) 
        self.joint2_f_subs = rospy.Subscriber('/joint2_f_controller/state', JointState, self.callback_joint2f) 
        rospy.spin()

        #Robot joints
        self.F_ARM_JOINTS = ['joint1_f','joint2_f']
        self.B_ARM_JOINTS = ['joint1_b','joint2_b']

        #Trajectory Duration
        self.trajectory_duration = rospy.rostime.Duration(0.8)
        self.rate = rospy.Rate(150)
        rospy.spin()

    #Callback function implementing the pose value received
    def callback_join1f(self,data):
        self.current_joint1f = data.current_pos
    def callback_join1b(self,data):
        self.current_joint1b = data.current_pos
    def callback_join2f(self,data):
        self.current_joint2f = data.current_pos
    def callback_join2b(self,data):
        self.current_joint2b = data.current_pos

    def joy_callback(self,data):
        left_horizontal_analog = data.axes[0]
        left_vertical_analog = data.axes[1]

        right_horizontal_analog = data.axes[2]
        right_vertical_analog = data.axes[3]

        button_analog_up = data.buttons[12]
        button_analog_down = data.buttons[13]

        if left_horizontal_analog != 0 :
            if left_horizontal_analog > 0.8:
                data = 1
                self.line_horizontal_control(data)
            if left_horizontal_analog < -0.8:
                data = -1
                self.line_horizontal_control(data)
    
        # if right_vertical_analog != 0:
        #     if right_vertical_analog > 0.8:
        #         data = 1
        #         self.arm_move(data)
        #     if right_vertical_analog < 0.8:
        #         data = 1
        #         self.arm_move(data)

        if data.button_analog_up or data.button_analog_down:
            if data.button_analog_up > 0:
                data = -1
                line_push_up(data)
            if data.button_analog_down > 0:
                data = 1
                line_push_up(data)

    def arm_move(self, data):
        key_vel = 3*data
        #F_ARM
        self.UP_F_ARM_POINTS = [ key_vel + self.current_joint1f , kel_vel +self.current_joint2f]
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
        self.UP_B_ARM_POINTS = [key_vel + self.current_joint1b , key_vel + self.current_joint2b]
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

    def traction_control(self, data):
        key_vel = 3*data.angular.z
        self.traction_f_publisher.publish(key_vel)
        self.traction_b_publisher.publish(key_vel)
        self.traction_ap_publisher.publish(-key_vel)

    def line_push_up(self, data):   
        goal_joint1f = self.joint1_f + 0.1*data
        self.joint1f_publisher.publish(goal_joint1f)
        self.rate.sleep()
        
if __name__ == '__main__':
    try:
        rospy.init_node('joy_controller', anonymous=True)
        x = arm_up_down_mimic_control()
    except rospy.ROSInterruptException: pass