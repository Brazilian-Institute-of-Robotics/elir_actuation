#!/usr/bin/env python
import rospy
from dynamixel_msgs.msg import JointState
from math import pi
from std_msgs.msg import Float64
from std_msgs.msg import String
from nav_msgs.msg import Odometry

#!/usr/bin/env python

class piro_odometry():
    def __init__(self,controller1_name,controller2_name,wheel_diameter):
        #Initiating odometry node
        # rospy.init_node('Odometry')
        #Odometry message object
        self.odometry_data = Odometry()
        # radius in meters
        self.wheel_radius = wheel_diameter/2
        #Subscribers of motors states
        self.tr1_subs = rospy.Subscriber(controller1_name +'/state', JointState, self.traction1_callback)
        self.tr2_subs = rospy.Subscriber(controller2_name +'/state', JointState, self.traction2_callback)
        self.odometry_traction_pub = rospy.Publisher('/odometry', Odometry, queue_size = 10)

        #Odometry value
        self.odometry_tr1 = 0.0
        self.odometry_tr2 = 0.0
        #previous_time_tr1 used in delta calculus
        self.previous_time_tr1 = 0.0
        self.previous_time_tr2 = 0.0
        #Variable auxiliar to acess a data from one callback function in the other
        self.odometry_aux = 0.0
        self.velocity_aux = 0.0

        rospy.spin()

    #Callback function implementing the JointState received
    def traction1_callback(self, data):
        angular_vel = data.velocity
        #Calculating linear velocity using the wheel radius
        linear_vel = angular_vel*self.wheel_radius
        #Estimating time to delta calculus for integration
        motor_time_sec = data.header.stamp.secs
        motor_time_nsec = data.header.stamp.nsecs
        motor_time = motor_time_sec + motor_time_nsec*1e-9

        delta_time = motor_time - self.previous_time_tr1
        #Calculating the distance diferential
        self.position_tr1_variation = linear_vel*delta_time
        #Implementing in the total distance
        self.odometry_tr1 = self.odometry_tr1 + self.position_tr1_variation
        #Reseting time to next distance diferential calculus
        self.previous_time_tr1 = motor_time
        #Calculating the final odometry_tr1
        odometry_final = (self.odometry_tr1 + self.odometry_aux)/2
        linear_vel_final = (linear_vel + self.velocity_aux)/2

        self.odometry_data.pose.pose.position.x = odometry_final
        self.odometry_data.twist.twist.linear.x = linear_vel_final
        self.odometry_data.header.stamp = rospy.Time.now()
        self.odometry_traction_pub.publish(self.odometry_data)

    def traction2_callback(self, data):
        #Due do the diferent spin direction of the servos, whe have to:
        angular_vel = -data.velocity
        #Calculating linear velocity using the wheel radius
        linear_vel = angular_vel*self.wheel_radius
        #Estimating time to delta calculus for integration
        motor_time_sec = data.header.stamp.secs
        motor_time_nsec = data.header.stamp.nsecs
        motor_time = motor_time_sec + motor_time_nsec*1e-9

        delta_time = motor_time - self.previous_time_tr2
        #Calculating the distance diferential
        self.position_tr2_variation = linear_vel*delta_time
        #Implementing in the total distance
        self.odometry_tr2 = self.odometry_tr2 + self.position_tr2_variation
        #Reseting time to next distance diferential calculus
        self.previous_time_tr2 = motor_time
        #Auxiliar variable for distance media
        self.odometry_aux = self.odometry_tr2
        #Auxiliar variable for velocity media
        self.velocity_aux = linear_vel

if __name__ == '__main__':
    x = piro_odometry("wheel_4_controller", "wheel_5_controller", 0.049)
