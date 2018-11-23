#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

class arm_mimic_control():

    def __init__(self):
        #Creating our node,publisher and subscriber
        self.b_arm_publisher = rospy.Publisher('/joint1_b_controller/command', Float64, queue_size=10)
        self.f_arm_publisher = rospy.Publisher('/joint1_f_controller/command', Float64, queue_size=10)
<<<<<<< Updated upstream

        self.b_arm_subscriber = rospy.Subscriber('/joint1_b_controller/state', JointState, self.callback_joint1b) 
        self.b_arm_subscriber = rospy.Subscriber('/joint1_f_controller/state', JointState, self.callback_joint1f)  

=======
        
        self.test_publisher = rospy.Publisher('/teste', String, queue_size=10)
        
        self.b_arm_subscriber = rospy.Subscriber('/joint1_b_controller/state', JointState, self.callback_joint1b) 
        self.f_arm_subscriber = rospy.Subscriber('/joint1_f_controller/state', JointState, self.callback_joint1f)  
        self.key_subscriber = rospy.Subscriber('/key_vel', Twist, self.callback)
        self.current_joint1b = 0
        self.current_joint1f = 0
>>>>>>> Stashed changes
        rospy.spin()
    #Callback function implementing the pose value received

    def callback_joint1b(self, msg):
        self.current_joint1b = msg.current_pos

<<<<<<< Updated upstream
    def callback(self, data):   
        key_vel = self.current_joint1b + 0.3 * data.angular.z
        if(key_vel >= 1.7 || key_vel <= -0.6):
            key_vel = self.current_joint1b
            self.b_arm_publisher.publish(key_vel)
            self.f_arm_publisher.publish(key_vel)
        else:
            self.b_arm_publisher.publish(key_vel)
            self.f_arm_publisher.publish(key_vel)
=======
    def callback_joint1f(self, msg):
        self.current_joint1f = msg.current_pos

    def callback(self, data):
        
        if data.angular.z == 0:
            pass
        
        if(self.current_joint1b == 0 and self.current_joint1f == 0):
           pass

        b1_goal = self.current_joint1b + 0.1 * data.angular.z
        f1_goal = self.current_joint1f + 0.1 * data.angular.z

        self.b_arm_publisher.publish(b1_goal)
        self.f_arm_publisher.publish(f1_goal)
>>>>>>> Stashed changes

if __name__ == '__main__':
    try:
        rospy.init_node('elir_line_controller', anonymous=True)
        x = arm_mimic_control()
    except rospy.ROSInterruptException: pass
