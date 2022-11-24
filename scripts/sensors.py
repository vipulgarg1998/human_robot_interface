#!/usr/bin/env python
import rospy
import euler2quat as e2q
from std_msgs.msg import Bool
from mavros_msgs.msg import StatusText
from sensor_msgs.msg import Imu
def status_callback(message):
    #get_caller_id(): Get fully resolved name of local node
    pub = rospy.Publisher('leak', Bool, queue_size=10)
    if "Leak" in message.text:
        rospy.loginfo(rospy.get_caller_id() + "there is a leak with severity =  ", message.severity)
        status = True
    else:
        rospy.loginfo(rospy.get_caller_id() + "No leak. Go Fish Captain!")
        status = False
    pub.publish(status)
def imu_callback(message):
    q = message.orientation
    [roll, pitch, yaw ] = e2q.euler_from_quaternion(q)
    print(roll,pitch,yaw)
def sensors():
    rospy.init_node('sensors', anonymous=True)

    rospy.Subscriber("/mavros/statustext/recv", StatusText, status_callback)
    rospy.Subscriber("/mavros/imu/data", Imu, imu_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    sensors()
