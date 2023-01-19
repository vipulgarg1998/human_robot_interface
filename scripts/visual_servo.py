#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
from cv2 import aruco
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from PID_controller import controllers

class VisualServo:
    def __init__(self):
        
        self.image_sub = rospy.Subscriber("/camera0/image_raw", Image, self.image_callback)

        self.surge_state_pub = rospy.Publisher('/surge/state', Float64, queue_size=10)
        self.surge_setpoint_pub = rospy.Publisher('/surge/setpoint', Float64, queue_size=10)
        self.heave_state_pub = rospy.Publisher('/heave/state', Float64, queue_size=10)
        self.heave_setpoint_pub = rospy.Publisher('/heave/setpoint', Float64, queue_size=10)
        self.sway_state_pub = rospy.Publisher('/sway/state', Float64, queue_size=10)
        self.sway_setpoint_pub = rospy.Publisher('/sway/setpoint', Float64, queue_size=10)
 
        # For OpenCV
        self.cv_bridge = CvBridge()
        self.cv_image = None

        # For Camera 
        self.caliberation_params_init = False
        self.fx = None
        self.fy = None 
        self.cx = None
        self.cy = None
        self.width = None
        self.height = None

    def image_callback(self, image_msg, view = False):
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        self.width = self.cv_image.shape[1]
        self.height = self.cv_image.shape[0]
        ids, corners = self.detectMarkers(self.cv_image)
        centroids = self.find_centroid(corners)
        self.publish_markers_state(ids, centroids)
        # Convert from BGR to RGB color format.
        if(view):
            cv2.imshow('image', self.cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.exit()

    def publish_markers_state(self, ids, corners):
        if(ids is None):
            return
        for i, id in enumerate(ids):
            self.control(corners[i][0], corners[i][1], id)

    def control(self, x, y, id):

        self.sway_setpoint_pub.publish(Float64(self.width/2))
        self.sway_state_pub.publish(Float64(y))

        if(id == 200):
            self.heave_setpoint_pub.publish(Float64(self.height/2))
            self.heave_state_pub.publish(Float64(x))
            
        print(f'Sending Control')
        

    def find_centroid(self, corners):
        centroids = []
        for corner in corners:
            x = np.mean(corner[0, :, 0])
            y = np.mean(corner[0, :, 1])
            centroids.append([x, y])
        return centroids
    
    def detectMarkers(self, image, view=True):
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        # print("parameters", parameters)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        corners = np.array(corners)
        
        if(view):
            frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
            cv2.imshow('aruco_image', frame_markers)
            cv2.waitKey(1)

        return ids, corners

    def exit(self):
        cv2.destroyAllWindows()
        
def main(args=None):
    # initializing the subscriber node
    rospy.init_node('visual_servo', anonymous=True)

    visual_servo = VisualServo()    
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass