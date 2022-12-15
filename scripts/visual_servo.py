#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
import cv2
from cv2 import aruco
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images


class VisualServo:
    def __init__(self):
        
        self.image_sub = rospy.Subscriber("/camera0/image_raw", Image, self.image_callback)
 
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

    def image_callback(self, image_msg, view = True):
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        self.width = self.cv_image.shape[1]
        self.height = self.cv_image.shape[0]
        self.detectMarkers(self.cv_image)
        # Convert from BGR to RGB color format.
        # cv2.imshow('image', self.cv_image)
        # #   out.write(nimg)
        # # Press `q` to exit.
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     self.exit()

    def find_centroid(self, corners):
        for corner in corners:
            x = np.mean(corner[0, :, 0])
            y = np.mean(corner[0, :, 1])
            print(f'X is {x} and Y is {y}')
    
    def detectMarkers(self, image, view=True):
        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = aruco.DetectorParameters_create()
        # print("parameters", parameters)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
        corners = np.array(corners)
        print("corners",corners.shape)
        self.find_centroid(corners)
        # print("ids", ids)
        # print("rejectedImgPoints",rejectedImgPoints)
        if(view):
            frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
            cv2.imshow('aruco_image', frame_markers)
            cv2.waitKey(1)
        
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