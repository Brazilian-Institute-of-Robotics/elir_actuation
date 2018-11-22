#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist

class line_mimic_control():

    def __init__(self):
        #Creating our node,publisher and subscriber
        self.key_subscriber = rospy.Subscriber('/key_vel', Twist, self.callback)
        self.b_arm_publisher = rospy.Publisher('/f_arm_trajectory_controller/command', Float64, queue_size=10)
        #self.traction_b_publisher = rospy.Publisher('/traction_b_controller/command', Float64, queue_size=10)
        #self.traction_ap_publisher = rospy.Publisher('/traction_ap_controller/command', Float64, queue_size=10)
       
        self.F_ARM_JOINTS = ['joint1_f','joint2_f']
        self.trajectory_duration = rospy.rostime.Duration(0.8)
        rospy.spin()
    #Callback function implementing the pose value received
    def callback(self, data):
        key_vel = 3*data.angular.z
       # self.b_arm_publisher.publish(key_vel)
        #self.traction_b_publisher.publish(key_vel)
        #self.traction_ap_publisher.publish(-key_vel)
         #Get current time for header stamp
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
        self.UP_F_ARM_POINTS = [key_vel, key_vel]
        msg.points.append(points )
        self.f_arm_publisher.publish(msg)
        rospy.loginfo("f_arm opened")

if __name__ == '__main__':
    try:
        rospy.init_node('elir_line_controller', anonymous=True)
        x = line_mimic_control()
    except rospy.ROSInterruptException: pass