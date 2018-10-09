#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



class claw_control_services():

    def __init__(self):
        #Creating our node,publisher and subscriber
        self.open_f_claw_service = rospy.Service('f_claw/open', Empty, self.open_f_claw)
        self.close_f_claw_service = rospy.Service('f_claw/close', Empty, self.close_f_claw)
        self.f_claw_publisher = rospy.Publisher('f_claw_trajectory_controller/command', JointTrajectory, queue_size=10)  

        self.open_b_claw_service = rospy.Service('b_claw/open', Empty, self.open_b_claw)
        self.close_b_claw_service = rospy.Service('b_claw/close', Empty, self.close_b_claw)
        self.b_claw_publisher = rospy.Publisher('b_claw_trajectory_controller/command', JointTrajectory, queue_size=10)

        self.open_ap_claw_service = rospy.Service('ap_claw/open', Empty, self.open_ap_claw)
        self.close_ap_claw_service = rospy.Service('ap_claw/close', Empty, self.close_ap_claw)
        self.ap_claw_publisher = rospy.Publisher('joint_claw_ap_controller/command', JointTrajectory, queue_size=10)
        
        #Robot joints
        self.F_CLAW_JOINTS = ['claw_f1','claw_f2']
        self.B_CLAW_JOINTS = ['claw_b1','claw_b2']
        self.AP_CLAW_JOINTS = ['claw_ap']
        #Opened and closed joint values
        self.OPEN_Q = [1.57,1.57]
        self.CLOSE_Q = [0,0]
        self.trajectory_duration = 0.7
        rospy.spin()
        

    def open_f_claw(self,req):
        #Get current time for header stamp
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.F_CLAW_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = rospy.rostime.Duration(2)
        points.positions = self.OPEN_Q
        #Append the trajectory points and publish
        msg.points.append(points)
        self.f_claw_publisher.publish(msg)
        rospy.loginfo("f_claw opened")

    def close_f_claw(self,req):
        #Get current time for header stamp
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.F_CLAW_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = rospy.rostime.Duration(0.5)
        points.positions = self.CLOSE_Q
        #Append the trajectory points and publish
        msg.points.append(points)
        self.f_claw_publisher.publish(msg)
        rospy.loginfo("f_claw closed")
    
    def open_b_claw(self,req):
        #Get current time for header stamp
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.B_CLAW_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = rospy.rostime.Duration(2)
        points.positions = self.OPEN_Q
        #Append the trajectory points and publish
        msg.points.append(points)
        self.b_claw_publisher.publish(msg)
        rospy.loginfo("b_claw opened")

    def close_b_claw(self,req):
        #Get current time for header stamp
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = self.B_CLAW_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = rospy.rostime.Duration(0.5)
        points.positions = self.CLOSE_Q
        #Append the trajectory points and publish
        msg.points.append(points)
        self.f_claw_publisher.publish(msg)
        rospy.loginfo("f_claw closed")

    def open_ap_claw(self,req):
        self.ap_claw_publisher.publish(1.57)

    def close_ap_claw(self,req)
        self.ap_claw_publisher.publish(0)
if __name__ == '__main__':
    try:
        rospy.init_node('claw_control_services', anonymous=True)
        x = claw_control_services()
    except rospy.ROSInterruptException: pass