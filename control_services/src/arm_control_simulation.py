#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import JointState

class arm_control_services():

    def __init__(self):
        #Creating our node,publisher and subscriber
        self.b_arm_subscriber = rospy.Subscriber('elir/joint_states', JointState, self.callback) 
        self.up_f_arm_service = rospy.Service('f_arm/up', int, self.up_f_arm)
        self.down_f_arm_service = rospy.Service('f_arm/down', int, self.down_f_arm)
        self.f_arm_publisher = rospy.Publisher('/elir/f_arm_trajectory_controller/command', JointTrajectory, queue_size=10)  
        self.up_b_arm_service = rospy.Service('b_arm/up', int, self.up_b_arm)
        self.down_b_arm_service = rospy.Service('b_arm/down', int, self.down_b_arm)
        self.b_arm_publisher = rospy.Publisher('/elir/b_arm_trajectory_controller/command', JointTrajectory, queue_size=10)  
        
        
        #Robot joints
        self.F_ARM_JOINTS = ['joint1_f','joint2_f']
        self.B_ARM_JOINTS = ['joint1_b','joint2_b']

        #Opened and closed joint values
        self.UP_F_ARM_POINTS = [-0.2, 0.2]
        self.UP_B_ARM_POINTS = [-0.2, 0.2]
        self.CLOSE_POINTS = [0 ,0]
        self.trajectory_duration = rospy.rostime.Duration(0.8)
        rospy.spin()
        self.outvalue = 0 


    def callback(self, msg):
        #get the current joint position 
        self.current_joint1b = msg.position[2]
        self.current_joint2b = msg.position[3]
        self.current_joint1f = msg.position[0]
        self.current_joint2f = msg.position[1]


    def up_f_arm(self,req):
        #Get current time for header stamp

        self.UP_F_ARM_POINTS = [-0.33 ]#+ self.current_joint1f , 0.33 +self.current_joint2f]
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.F_ARM_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = self.trajectory_duration
        points.positions = self.UP_F_ARM_POINTS
        rospy.loginfo("Joint position received:"+str(points.positions))
        #Append the trajectory points and publish
        msg.points.append(points)
        self.f_arm_publisher.publish(msg)
        rospy.loginfo("f_arm up")
        

    def down_f_arm(self,req):
        self.CLOSE_POINTS = [0.2 ]#+ self.current_joint1f , -0.2+self.current_joint2f]

        #Get current time for header stamp
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.F_ARM_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = self.trajectory_duration
        points.positions = self.CLOSE_POINTS
        #Append the trajectory points and publish
        msg.points.append(points)
        self.f_arm_publisher.publish(msg)
        rospy.loginfo("f_arm down")
    
    def up_b_arm(self,req):
        self.UP_B_ARM_POINTS = [-0.33 + self.current_joint1b , 0.2 + self.current_joint2b]
        

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
        rospy.loginfo("b_arm up")
        

    def down_b_arm(self,req):
        self.CLOSE_POINTS = [0.2 + self.current_joint1b , -0.2 + self.current_joint2b]

        #Get current time for header stamp
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.B_ARM_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = self.trajectory_duration
        points.positions = self.CLOSE_POINTS
        #Append the trajectory points and publish
        msg.points.append(points)
        self.b_arm_publisher.publish(msg)
        rospy.loginfo("b_arm down")

if __name__ == '__main__':
    try:
        rospy.init_node('arm_control_services', anonymous=True)
        x = arm_control_services()
    except rospy.ROSInterruptException: pass