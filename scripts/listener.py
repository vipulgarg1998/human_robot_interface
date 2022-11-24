#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from mavros_msgs.msg import StatusText
def chatter_callback(message):
    #get_caller_id(): Get fully resolved name of local node
    pub = rospy.Publisher('leak', Bool, queue_size=10)
    if "Leak" in message.text:

        rospy.loginfo(rospy.get_caller_id() + "there is a leak with severity =  ", message.severity)
        status = True
    else:
        rospy.loginfo(rospy.get_caller_id() + "No leak. Go Fish Captain!")
        status = False
    pub.publish(status)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/mavros/statustext/recv", StatusText, chatter_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
