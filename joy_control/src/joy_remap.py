#!/usr/bin/env python
import rospy
from std_msgs.msg  import Float64
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from dynamixel_msgs.msg import JointState
from sensor_msgs.msg import Joy
class joy_control():

    def __init__(self):

        #self.joint1_b_subs = rospy.Subscriber('/joint1_b_controller/state', JointState, self.callback_joint1b) 
        #self.joint1_f_subs = rospy.Subscriber('/joint1_f_controller/state', JointState, self.callback_joint1f) 
        #self.joint2_b_subs = rospy.Subscriber('/joint2_b_controller/state', JointState, self.callback_joint2b) 
        #self.joint2_f_subs = rospy.Subscriber('/joint2_f_controller/state', JointState, self.callback_joint2f) 

        self.joy_subscriber = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.joint1f_publisher = rospy.Publisher('/joint1_f_controller/command', Float64, queue_size=10)
        self.joint1b_publisher = rospy.Publisher('/joint1_b_controller/command', Float64, queue_size=10)
        self.joint2f_publisher = rospy.Publisher('/joint2_f_controller/command', Float64, queue_size=10)
        self.joint2b_publisher = rospy.Publisher('/joint2_b_controller/command', Float64, queue_size=10)  

        self.traction_f_publisher = rospy.Publisher('/traction_f_controller/command', Float64, queue_size=10)
        self.traction_b_publisher = rospy.Publisher('/traction_b_controller/command', Float64, queue_size=10)
        self.traction_ap_publisher = rospy.Publisher('/traction_ap_controller/command', Float64, queue_size=10)


        #Trajectory Duration
        self.trajectory_duration = rospy.rostime.Duration(0.8)

        self.current_joint1b = 0.7
        self.current_joint1f = 0.7
        self.current_joint2b = -0.7
        self.current_joint2f = -0.7
        self.f_arm_pos_increment = 0
        self.b_arm_pos_increment = 0

        self.rate = rospy.Rate(30)

    #Callback function implementing the pose value received
    def callback_joint1f(self,data):
        self.current_joint1f = data.current_pos

    def callback_joint1b(self,data):
        self.current_joint1b = data.current_pos

    def callback_joint2f(self,data):
        self.current_joint2f = data.current_pos

    def callback_joint2b(self,data):
        self.current_joint2b = data.current_pos

    def joy_callback(self,data):

        left_horizontal_analog = data.axes[0]
        left_vertical_analog = data.axes[1]

        right_horizontal_analog = data.axes[2]
        right_vertical_analog = data.axes[3]

        button_analog_up = data.buttons[12]
        button_analog_down = data.buttons[13]

        button_analog_right =data.buttons[15]
        button_analog_left =data.buttons[14]

        right_trigger = data.buttons[7]
        left_trigger = data.buttons[6]
        
        if right_trigger == 1:
            self.move_joint_2f = True
        else:
            self.move_joint_2f = False
        
        if left_trigger == 1:
            self.move_joint_2b = True
        else:
            self.move_joint_2b= False

        #Define a variavel de incremento no braco f
        if right_vertical_analog != 0 :
            if right_vertical_analog > 0.8:
                data = 1
                self.f_arm_pos_increment = -data
            if right_vertical_analog < -0.8:
                data = -1
                self.f_arm_pos_increment = -data
        else:
            self.f_arm_pos_increment = 0
            
        
        #Define a variavel de incremento no braco b
        if left_vertical_analog != 0 :
            if left_vertical_analog > 0:
                data = 1
                self.b_arm_pos_increment = -data
            if left_vertical_analog < 0:
                data = -1
                self.b_arm_pos_increment = -data
        else:
            self.b_arm_pos_increment = 0

        #Chama a funcao para mover na linha quando botao for pressionado
        if button_analog_left or button_analog_right :
            if button_analog_left:
                data = 1
                self.line_horizontal_control(data)
            if button_analog_right:
                data = -1
                self.line_horizontal_control(data)
        else:
            self.line_horizontal_control(0)

    def traction_control(self, data):
        key_vel = 3*data.angular.z
        self.traction_f_publisher.publish(key_vel)
        self.traction_b_publisher.publish(key_vel)
        self.traction_ap_publisher.publish(-key_vel)

    def f_arm_move(self,data):
        if not self.move_joint_2f:
            self.goal_joint_1f = self.current_joint1f + 0.3*data
            #self.joint1f_publisher.publish(goal_joint_1f)
            print('Movendo junta 1f')
            print('Valor atual da junta:' , self.current_joint1f)
            self.rate.sleep()
        else:
            self.goal_joint_2f = self.current_joint2f + 0.1*data
            #self.joint2f_publisher.publish(goal_joint_2f)
            print('Movendo junta 2f')
            print('Valor atual da junta:' , self.current_joint2f)
            self.rate.sleep()


    def b_arm_move(self,data):
        if not self.move_joint_2b:
            goal_joint_1b = self.current_joint1b + 0.005*data
            self.joint1b_publisher.publish(goal_joint_1b)
            print('Movendo junta 1b')
            print('Valor atual da junta:' , self.current_joint1b)
            #self.current_joint1b = goal_joint_1b
            self.rate.sleep()
        else:
            goal_joint_2b = self.current_joint2b + 0.3*data
            self.joint2b_publisher.publish(goal_joint_2b)
            print('Movendo junta 2b')
            print('Valor atual da junta:' , self.current_joint2b)
            self.rate.sleep()

    def line_horizontal_control(self, data):
        key_vel = 3*data
        self.traction_f_publisher.publish(key_vel)
        self.traction_b_publisher.publish(key_vel)
        self.traction_ap_publisher.publish(-key_vel)

if __name__ == '__main__':
    try:
        rospy.init_node('joy_controller', anonymous=True)
        x = joy_control()
        while not rospy.is_shutdown():
            if x.f_arm_pos_increment != 0:
                data = x.f_arm_pos_increment
                x.f_arm_move(data)
            if x.b_arm_pos_increment != 0:
                print(x.b_arm_pos_increment)
                data = x.b_arm_pos_increment
                x.b_arm_move(data)
    except rospy.ROSInterruptException: pass 
### COMANDOS 
#ANALOGICOS 
#DIREITO/ESQUERDO SEM O GATILHO SOBE JUNTA 1F/1B, 
#COM GATILHO CORRESPONDENTE JUNTA 2F/2B

##ANALOGICO DE BOTAO
#DIREITA ROBO PRA DIREITA ESQUERDA ROBO PRA ESQUERDA


    