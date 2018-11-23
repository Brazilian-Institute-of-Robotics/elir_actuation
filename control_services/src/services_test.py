#!/usr/bin/env python

import sys
import rospy
from std_srvs.srv import Empty
from dynamixel_controllers.srv import TorqueEnable
from std_srvs.srv import Empty

def close_all_claws_client():
   try:
        close_b_claw = rospy.ServiceProxy('b_claw/close', Empty)
        close_ap_claw = rospy.ServiceProxy('ap_claw/close', Empty)
        close_f_claw = rospy.ServiceProxy('f_claw/close', Empty)
        close_b_claw()
        close_ap_claw()
        close_f_claw()

   except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def open_b_claw_client():
   try:
        open_b_claw = rospy.ServiceProxy('b_claw/open', Empty)
        open_b_claw()

   except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def open_ap_claw_client():
   try:
        open_ap_claw = rospy.ServiceProxy('ap_claw/open', Empty)
        open_ap_claw()

   except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def open_f_claw_client():
   try:
        open_f_claw = rospy.ServiceProxy('f_claw/open', Empty)
        open_f_claw()

   except rospy.ServiceException, e:
        print "Service call failed: %s"%e


def torque_enable_client():
   try:
        torque_1b = rospy.ServiceProxy('joint1_b_controller/torque_enable', TorqueEnable)
        torque_2b = rospy.ServiceProxy('joint2_b_controller/torque_enable', TorqueEnable)
        torque_1f = rospy.ServiceProxy('joint1_f_controller/torque_enable', TorqueEnable)
        torque_2f = rospy.ServiceProxy('joint2_f_controller/torque_enable', TorqueEnable)
      
        torque_1b(1)
        torque_2b(1)
        torque_1f(1)
        torque_2f(1)
      

   except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def torque_disable_client():
   try:
        torque_1b = rospy.ServiceProxy('joint1_b_controller/torque_enable', TorqueEnable)
        torque_2b = rospy.ServiceProxy('joint2_b_controller/torque_enable', TorqueEnable)
        torque_1f = rospy.ServiceProxy('joint1_f_controller/torque_enable', TorqueEnable)
        torque_2f = rospy.ServiceProxy('joint2_f_controller/torque_enable', TorqueEnable)
      
        torque_1b(0)
        torque_2b(0)
        torque_1f(0)
        torque_2f(0)
     
      
   except rospy.ServiceException, e:
        print "Service call failed: %s"%e



def up_b_arm_client():
   try:
        up_b_arm = rospy.ServiceProxy('/b_arm/up', Empty)
        up_b_arm()
      
   except rospy.ServiceException, e:
       print "Service call failed: %s"%e

def down_b_arm_client():
   try:
        down_b_arm = rospy.ServiceProxy('/b_arm/down', Empty)
        down_b_arm()
      
   except rospy.ServiceException, e:
       print "Service call failed: %s"%e

def up_f_arm_client():
   try:
        up_f_arm = rospy.ServiceProxy('/f_arm/up', Empty)
        up_f_arm()
      
   except rospy.ServiceException, e:
       print "Service call failed: %s"%e

def down_f_arm_client():
   try:
        down_f_arm = rospy.ServiceProxy('/f_arm/down', Empty)
        down_f_arm()
      
   except rospy.ServiceException, e:
       print "Service call failed: %s"%e

if __name__ == "__main__":
    print "============ Press `Enter` to enable the claw torque  ..."
    raw_input()
    close_all_claws_client()

    print "============ Press `Enter` to enable the claw torque  ..."
    raw_input()
    torque_enable_client()

    print "============ Press `Enter` to disable the claw torque  ..."
    raw_input()
    up_b_arm_client()

    print "============ Press `Enter` to enable the claw torque  ..."
    raw_input()
    open_b_claw_client()

    print "============ Press `Enter` to enable the claw torque  ..."
    raw_input()
    close_all_claws_client()

    print "============ Press `Enter` to disable the claw torque  ..."
    raw_input()
    down_b_arm_client()
    
    print "============ Press `Enter` to disable the claw torque  ..."
    raw_input()
    torque_disable_client()



    while(service != 0):
        print "============ Pressione `0` para sair"
        print "============ Pressione `1` para desabilitar o torque das juntas 1f 2f 1b 2b"
        print "============ Pressione `2` para habilitar o torque das juntas 1f 2f 1b 2b""
        print "============ Pressione `3` para fechar todas as garras"
        print "============ Pressione `4` para abrir a garra de tras ..."
        print "============ Pressione `5` para abrir a garra da frente ..."
        print "============ Pressione `6` para abrir a garra do meio ..."
        print "============ Pressione `7` para subir o braço da tras ..."
        print "============ Pressione `8` para descer o braço de tras ..."
        print "============ Pressione `9` para subir o braço de frente..."
        print "============ Pressione `10` para descer o braço de frente ..."


        service = input('Insira qual serviço:')

        if service == 0:
            print("byebye")

        elif service == 1:
            torque_disable_client()

        elif service == 2:
            torque_enable_client()

        elif service == 3:
            close_all_claws_client()

        elif service == 4:
            open_b_claw_client()

        elif service == 5:
            open_f_claw_client

        elif service == 6:
            open_ap_claw_client

        elif service == 7:
            up_b_arm_client()
        elif service == 8:
            down_b_arm_client()
        elif service == 9:
            up_f_arm_client()
        elif service == 10:
            down_f_arm_client()
            

        
            

    




