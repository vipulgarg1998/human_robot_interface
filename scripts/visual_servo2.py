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

        self.exit_flag = False
        self.centroids = None
        self.kp = 0.0003
        self.kd = 0.0001*0
        self.move = True
        self.ids = []
        self.detected = False
        self.Ep = 0
        self.Ep0 = 0
        self.dt = 1/2.0
        self.corners70d =  np.array([[179,196] , [138,199] , [133,157] , [173,153]])
        self.corners200d =  np.array([[171,138], [132,143] , [128,103] , [166,98]])
        self.detected70 = False
        self.detected200 = False
        # [[ 70]
        #     [200]] [[[[179. 196.]
        #     [138. 199.]
        #     [133. 157.]
        #     [173. 153.]]]


        #     [[[171. 138.]
        #     [132. 143.]
        #     [128. 103.]
        #     [166.  98.]]]]

        self.corners70 = self.corners70d
        self.corners200 = self.corners200d
    def image_callback(self, image_msg, view = False):
        self.cv_image = self.cv_bridge.imgmsg_to_cv2(image_msg)
        self.width = self.cv_image.shape[1]
        self.height = self.cv_image.shape[0]

        self.draw_ref_features()
        self.draw_detected_features()

        ids, corners = self.detectMarkers(self.cv_image)
        centroids = self.find_centroid(corners)
        self.calculate_error()
        # self.publish_markers_state(ids, centroids)
        # Convert from BGR to RGB color format.
        self.obtain_features(corners,ids)
        if(view):
            cv2.imshow('image', self.cv_image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.exit()


    def calculate_error(self):
        self.Epx = 0
        self.Epy = 0
        self.referenceX = 0
        self.referenceY = 0
        self.currentX = 0 
        self.currentY = 0
        # if self.detected70 == True:
        #     self.Epx = self.Epx + np.mean((self.corners70d - self.corners70)[:,0])
        #     self.Epy = self.Epy + np.mean((self.corners70d - self.corners70)[:,1])

        # if self.detected200 == True:
        #     self.Epx = self.Epx + np.mean((self.corners200d - self.corners200)[:,0])
        #     self.Epy = self.Epy + np.mean((self.corners200d - self.corners200)[:,1])
        if self.detected70 == True:
            self.referenceX = self.referenceX + np.mean((self.corners70d)[:,0])
            self.currentX = self.currentX + np.mean((self.corners70)[:,0])
            self.referenceY = self.referenceY + np.mean((self.corners70d)[:,1])
            self.currentY = self.currentY + np.mean((self.corners70)[:,1])
        if self.detected200 == True:
            self.referenceX = self.referenceX + np.mean((self.corners200d)[:,0])
            self.currentX = self.currentX + np.mean((self.corners200)[:,0])
            self.referenceY = self.referenceY + np.mean((self.corners200d)[:,1])
            self.currentY = self.currentY + np.mean((self.corners200)[:,1])
        if self.detected70 == False and self.detected200 == False:
            self.heave_state_pub.publish(Float64(self.currentY))
            self.heave_setpoint_pub.publish(Float64(self.referenceY))
            self.sway_state_pub.publish(Float64(self.currentX))
            self.sway_setpoint_pub.publish(Float64(self.currentX))
        print(f"self.Epx: {self.Epx}")
        print(f"self.Epy: {self.Epy}")

    def obtain_features(self,corners,ids):
        self.corners70 = self.corners70d
        self.corners200 = self.corners200d


        self.detected70 = False
        self.detected200 = False


        if self.detected == True:
            for i, id in enumerate(ids):
                if id[0] == 70:
                    self.corners70 = np.array(corners[i][0])
                    self.detected70 = True
                    print(f"corners70 = {self.corners70}")

                elif id[0] == 200:
                    self.corners200 = np.array(corners[i][0])
                    self.detected200 = True
                    print(f"corners200 = {self.corners200}")

        
                

    def publish_markers_state(self, ids, corners):
        if(ids is None):
            return
        for i, id in enumerate(ids):
            if(id == 70):
                self.control(corners[i][0], corners[i][1])

    def control(self, x, y):
        controllers.update_centroid(x,y)
        Ex = controllers.obtain_commands_yaw()
        Ey = controllers.obtain_commands_heave()
        print(Ex, Ey)
    def draw_ref_features(self):
        radius = 10
        color = (0, 255, 0)
        thickness = 4
        for p in self.corners70d:
            center_coordinates = (p[0], p[1])
            self.cv_image = cv2.circle(self.cv_image, center_coordinates, radius, color, thickness)
        for p in self.corners200d:
            center_coordinates = (p[0], p[1])
            self.cv_image = cv2.circle(self.cv_image, center_coordinates, radius, color, thickness)
    def draw_detected_features(self):
        radius = 10
        color = (0, 0, 255)
        thickness = 2
        if self.detected70 == True:
            for p in self.corners70:
                center_coordinates = (p[0], p[1])
                self.cv_image = cv2.circle(self.cv_image, center_coordinates, radius, color, thickness)
        if self.detected200 == True:
            for p in self.corners200:
                center_coordinates = (p[0], p[1])
                self.cv_image = cv2.circle(self.cv_image, center_coordinates, radius, color, thickness)
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
        #print(corners)
        if corners.size  != 0:
            self.detected = True        #if it detected something
        else:
            self.detected = False
        if(view):
            frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
            cv2.imshow('aruco_image', frame_markers)
            cv2.waitKey(1)
        # print(ids)
        # print(ids,corners)
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