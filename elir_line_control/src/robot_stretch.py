#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class arm_mimic_control():

    def __init__(self):
        #Creating our node,publisher and subscriber
        self.key_subscriber = rospy.Subscriber('/key_vel', Twist, self.callback)
        self.b_arm_publisher = rospy.Publisher('/b_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.f_arm_publisher = rospy.Publisher('/f_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
        self.b_arm_subscriber = rospy.Subscriber('/joint1_b_controller/state', JointState, self.callback_joint1b)  
        self.b_arm_subscriber = rospy.Subscriber('/joint1_f_controller/state', JointState, self.callback_joint1f) 
        #self.joints_subscriber = rospy.Subscriber('/elir/joint_states', JointState, self.callback_joints) 
        self.traction_f_publisher = rospy.Publisher('/traction_f_controller/command', Float64, queue_size=10)
        self.traction_b_publisher = rospy.Publisher('/traction_b_controller/command', Float64, queue_size=10)
        self.f_arm_joints = ['joint1_f', 'joint2_f']
        self.b_arm_joints = ['joint1_b', 'joint2_b']
        self.trajectory_duration = rospy.rostime.Duration(0.8)
        rospy.spin()

    #Callback function implementing the pose value received
    #def callback_joints(self, msg):
        #self.current_joint1b = msg.position[2]
        #self.current_joint1f = msg.position[3] 


    def callback_joint1b(self, msg):
        self.current_joint1b = msg.current_pos


    def callback_joint1f(self, msg):
        self.current_joint1f = msg.current_pos

    def callback(self, data):
        
        if data.angular.z == 0:
            pass

        if(self.current_joint1b == 0 and self.current_joint1f == 0):       
            pass
        
        time = rospy.get_time()
        ####################################B_ARM MESSAGE CONFIG####################################
        msg_to_b_arm = JointTrajectory()                    #-Creates a Joint Trajectory msg to be published on Back Arm-#
        msg_to_b_arm.joint_names = self.b_arm_joints        #-Get joint names to back arm-#
        msg_to_b_arm.points = []                            #-Create points array to back arm-#
        points_to_b_arm = JointTrajectoryPoint()            #-Create point object to be published on back arm-#
        points_to_b_arm.time_from_start = self.trajectory_duration  #-Get time to back arm trajectory duration-#
        
        ###################################F_ARM MENSSAGE CONFIG####################################
        msg_to_f_arm = JointTrajectory()                    #-Creates a Joint Trajectory msg to be published on Front Arm-#
        msg_to_f_arm.joint_names = self.f_arm_joints        #-Get joint names to front arm-#
        msg_to_f_arm.points = []                            #-Create points array to front arm-#   
        points_to_f_arm = JointTrajectoryPoint()            #-Create point object to be published on front arm-#         
        points_to_f_arm.time_from_start = self.trajectory_duration    #-Get time to front arm trajectory duration-#
        
        ###########################################################
        key_vel = self.current_joint1b + 2 * data.angular.z         #-Get and convert keyboard input-#
        key_vel_to_wheels = 3 * data.angular.z
        points_to_f_arm.positions = [key_vel, -key_vel]
        points_to_b_arm.positions = [key_vel, -key_vel]
        msg_to_b_arm.points.append(points_to_b_arm)
        msg_to_f_arm.points.append(points_to_f_arm)

        if(key_vel >= 30 or key_vel <= -30):
            key_vel = self.current_joint1b
            self.b_arm_publisher.publish(msg_to_b_arm)
            self.f_arm_publisher.publish(msg_to_f_arm)
            self.traction_f_publisher.publish(key_vel_to_wheels)
            self.traction_b_publisher.publish(-key_vel_to_wheels)
        else:
            self.b_arm_publisher.publish(msg_to_b_arm)
            self.f_arm_publisher.publish(msg_to_f_arm)
            self.traction_f_publisher.publish(key_vel_to_wheels)
            self.traction_b_publisher.publish(-key_vel_to_wheels)

if __name__ == '__main__':
    try:
        rospy.init_node('elir_line_controller', anonymous=True)
        x = arm_mimic_control()
    except rospy.ROSInterruptException: pass