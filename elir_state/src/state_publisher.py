#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from dynamixel_msgs.msg import MotorState,MotorStateList
from math import pi
import time

rospy.init_node('State_Publisher')
register_to_rad = (360.0/4095)*(pi/180)

#Dictionary containing the joint name and master servo ID
joints = {11:'joint1_f',12:'joint2_f',13:'joint1_b',14:'joint2_b',
31:'joint_garra_tracao_f1',41:'joint_garra_tracao_f2',131:'joint_garra_tracao_b1',
141:'joint_garra_tracao_b2',71:'joint_garra_tracao_ap',51:'joint_eixo_tracao_f1',
61:'joint_eixo_tracao_f2',151:'joint_eixo_tracao_b1',161:'joint_eixo_tracao_b2',
81:'joint_eixo_tracao_ap'
}

#Offset of the joints, configured in the controllers config files
offset = {'joint1_f':2380,'joint2_f':2791,'joint1_b':0,'joint2_b':0,
'joint_garra_tracao_f1':0,'joint_garra_tracao_f2':0,'joint_garra_tracao_b1':0,
 'joint_garra_tracao_b2':0, 'joint_garra_tracao_ap':0,'joint_eixo_tracao_f1':0,
'joint_eixo_tracao_f2':0,'joint_eixo_tracao_b1':0,'joint_eixo_tracao_b2':0,
'joint_eixo_tracao_ap':0
}

'''
Function: process(). Callback for subscriber of raw data from dynamixel motor. 
Logic: Dynamixel position = 0 for 0 degree and Dynamixel position = 1023 for 300 degree.
       Current position can be calculated by (position*(300.0/1023))*(pi/180) radian.
       Where position = feddback-offset.
'''
def process(msg):
	joint_states = JointState()

	joint_states.header.stamp = rospy.Time.now()
	#Creates fake values for joints
	for i in joints:
		if not (i==11 or i ==12):
			joint_states.name.append(joints[i])
			joint_states.position.append(0)
			joint_states.velocity.append(0)

	for servo in msg.motor_states:
		#Ignore slave servos
		if servo.id in joints:
			joint_name = joints[servo.id]
			joint_offset = offset[joint_name]
			joint_position = (servo.position - joint_offset)*register_to_rad
			joint_velocity = servo.speed
			print(joints[servo.id])
			print("Position",servo.position)
			print("Offset:",joint_offset)
			print("Position moveit",joint_position)
			joint_states.name.append(joint_name)
			joint_states.position.append(joint_position)
			joint_states.velocity.append(joint_velocity)
	pub.publish(joint_states)

# Subscriber for raw feedback from dynamixel motor. Position of the motor will be in the range of (0,1023).
sub = rospy.Subscriber('/motor_states/pan_tilt_port',MotorStateList,process)
# Publisher for the current position of dynamixel motor in radian
pub = rospy.Publisher('/robot/joint_states',JointState,queue_size=10)

rospy.spin()
