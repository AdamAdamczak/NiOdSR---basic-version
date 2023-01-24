#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import CameraInfo


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.window_name = "camera"
        self.subscription = self.create_subscription(Image,'image_raw',self.listener_callback1,10)
        self.subscription = self.create_subscription(CameraInfo,'/camera_info',self.listener_callback2,10)
        self.publisher_ = self.create_publisher(Point, '/marker_point', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.point = []
        self.width=512
        self.hight=700
        self.dist_param=None
        self.matrix=None
        self.marker_size=10

        
    def listener_callback2(self,camera_info):
        self.dist_param=camera_info.d
        self.matrix=camera_info.k
        print(self.dist_param)
        print(self.matrix)

    def listener_callback1(self, image_data):
        image = CvBridge().imgmsg_to_cv2(image_data,"bgr8")
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids,rejected) = cv2.aruco.detectMarkers(image, arucoDict,parameters=arucoParams)
        cv2.aruco.drawDetectedMarkers(image,corners,ids)
        
        cv_image = np.zeros((512,700,3), np.uint8)
        
        first_line=cv2.line(image, (int(self.hight/2),0), (int(self.hight/2),self.width), (0,255,0), 5)
        sec_line=cv2.line(image, (0,int(self.width/2)), (self.hight,int(self.width/2)), (0,255,0), 5)           
        cv2.imshow(self.window_name, sec_line)

        if len(corners)>0:  
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
                
            x_centerPixel = x_sum*.25   
            y_centerPixel = y_sum*.25
            self.point=[x_centerPixel,y_centerPixel,0]
        cv2.waitKey(10)

    def timer_callback(self):
        msg=Point()
        if len(self.point)>0:
          msg.x=float(self.point[0])
          msg.y=float(self.point[1])
          self.publisher_.publish(msg)
        self.i += 1
    



def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
