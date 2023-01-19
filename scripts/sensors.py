#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import keyboard
from mavros_msgs.msg import StatusText
from sensor_msgs.msg import Imu
from mavros_msgs.msg import VFR_HUD
from mavros_msgs.msg import OverrideRCIn


class Sensors:
    def __init__(self, debug = False):
        self.debug = debug

        rospy.Subscriber("/mavros/statustext/recv", StatusText, self.status_callback)
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.depth_callback)

        self.leak_pub = rospy.Publisher('leak', Bool, queue_size=10)
        self.depth_pub = rospy.Publisher('depth_topic', Float64, queue_size=10)

    def status_callback(self, message):
        if "Leak" in message.text:
            rospy.loginfo(rospy.get_caller_id() + " there is a leak with severity =  ", message.severity)
            status = True
        else:
            rospy.loginfo(rospy.get_caller_id() + " No leak. Go Fish Captain!")
            status = False
        self.leak_pub.publish(status)

    def imu_callback(self, message):
        q = message.orientation
        [roll, pitch, yaw ] = self.euler_from_quaternion(q)
        if(self.debug):
            print(f"roll: {roll}, pitch: {pitch}, yaw:{yaw}")

    def depth_callback(self, message):
        altitude = message.altitude
        self.depth_pub.publish(altitude)
        if(self.debug):
            print(f"Depth is: {altitude}")

    def spin(self):
        rospy.spin()

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

def main():
    rospy.init_node('sensors', anonymous=True)
    sensors = Sensors()
    sensors.spin()

if __name__ == '__main__':
    main()