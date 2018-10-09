#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

F_CLAW_JOINTS = ['claw_f1','claw_f2']
B_CLAW_JOINTS = ['claw_b1','claw_b2']
AP_CLAW_JOINTS = ['claw_ap']

OPEN_Q = [-1.57,1.57]
CLOSE_Q = [0, 0]

class claw_control_services():

    def __init__(self):
        #Creating our node,publisher and subscriber
        self.open_f_claw_service = rospy.Service('f_claw/open', Empty, self.open_f_claw)
        self.close_f_claw_service = rospy.Service('f_claw/close', Empty, self.close_f_claw)
        self.f_claw_publisher = rospy.Publisher('/f_claw_trajectory_controller/command', JointTrajectory, queue_size=10)  

        #self.open_b_claw_service = rospy.Service('b_claw/open', Empty, self.open_b_claw)
        #self.close_b_claw_service = rospy.Service('b_claw/close', Empty, self.close_b_claw)

        #self.open_ap_claw_service = rospy.Service('ap_claw/open', Empty, self.open_ap_claw)
        #self.close_ap_claw_service = rospy.Service('ap_claw/close', Empty, self.close_ap_claw)

        rospy.spin()
        
    #Callback function implementing the pose value received
    def open_f_claw(self,req):
        #Get current time for header stamp
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = F_CLAW_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = rospy.rostime.Duration(2)
        points.positions = OPEN_Q

        msg.points.append(points)
        self.f_claw_publisher.publish(msg)
        rospy.loginfo("f_claw Opened")

    def close_f_claw(self,req):
        #Get current time for header stamp
        time = rospy.get_time()
        #Joint Trajectory message
        msg = JointTrajectory()
        msg.joint_names = F_CLAW_JOINTS
        msg.points = []
        #Joint Trajectory points
        points = JointTrajectoryPoint()
        points.time_from_start = rospy.rostime.Duration(2)
        points.positions = CLOSE_Q

        msg.points.append(points)
        self.f_claw_publisher.publish(msg)
        rospy.loginfo("f_claw CLOSED")

if __name__ == '__main__':
    try:
        rospy.init_node('claw_control_services', anonymous=True)
        x = claw_control_services()
    except rospy.ROSInterruptException: pass