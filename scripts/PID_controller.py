#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn

class yaw_controller:

    def __init__(self):
        rospy.init_node('pid_controller', anonymous=True)
        self.publisher = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

        self.x = 0
        self.y = 0
        self.xd = 0
        self.yd = 0
        self.kp = 10

        self.kd = 0
        self.ki = 0
        self.dt = 1/30
        self.e_x_I = 0
        self.e_x_dot = 0
        self.e_x = 0
    def update_centroid(self,x,y):
        self.x = x
        self.y = y

    def obtain_commands(self):
        self.e_x = self.xd - self.x
        self.e_x_dot = (self.e_x-self.e_x0)/self.dt
        self.e_x0 = self.e_x
        self.e_x_I = self.e_x_I + self.e_x*self.dt

        E = self.kp * self.e_x + self.kd * self.e_x_dot + self.ki * self.e_x_I
        print(E)


        
