#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Empty
from dynamixel_controllers.srv import TorqueEnable

def torque_enable_client():
    try:
        service_f1 = rospy.ServiceProxy('joint_claw_f1_controller/torque_enable', TorqueEnable)
        service_f2 = rospy.ServiceProxy('joint_claw_f2_controller/torque_enable', TorqueEnable)
        service_ap = rospy.ServiceProxy('joint_claw_ap_controller/torque_enable', TorqueEnable)
        service_b1 = rospy.ServiceProxy('joint_claw_b1_controller/torque_enable', TorqueEnable)
        service_b2 = rospy.ServiceProxy('joint_claw_b2_controller/torque_enable', TorqueEnable)
        
        service_b1(1)
        service_b2(1)
        service_ap(1)
        service_f1(1)
        service_f2(1)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def torque_disable_client():
    try:
        service_f1 = rospy.ServiceProxy('joint_claw_f1_controller/torque_enable', TorqueEnable)
        service_f2 = rospy.ServiceProxy('joint_claw_f2_controller/torque_enable', TorqueEnable)
        service_ap = rospy.ServiceProxy('joint_claw_ap_controller/torque_enable', TorqueEnable)
        service_b1 = rospy.ServiceProxy('joint_claw_b1_controller/torque_enable', TorqueEnable)
        service_b2 = rospy.ServiceProxy('joint_claw_b2_controller/torque_enable', TorqueEnable)
        
        service_b1(0)
        service_b2(0)
        service_ap(0)
        service_f1(0)
        service_f2(0)
        
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

        

if __name__ == "__main__":
    
    print "============ Press `Enter` to enable the claw torque  ..."
    raw_input()
    torque_enable_client()

    print "============ Press `Enter` to disable the claw torque  ..."
    raw_input()
    torque_disable_client()




