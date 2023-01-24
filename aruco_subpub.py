#!/usr/bin/env python3
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # ROS2 package to convert between ROS and OpenCV Images
import cv2 # Python OpenCV library
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist,Vector3




class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Point,'/marker_point',self.listener_callback,10)
        self.subscription  
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.width=512
        self.hight=700
        self.vel_x=0
        self.vel_y=0


    def listener_callback(self, marker_pos):
        if marker_pos.x>int(self.width/4*3):
            self.vel_x=1.0
        if marker_pos.x<int(self.width/4):
            self.vel_x=-1.0
        if marker_pos.x>int(self.width/4) and marker_pos.x<int(self.width/4*3):
            self.vel_x=0
            
            
        if marker_pos.y>int(self.width/4*3):
            self.vel_y=1.0
        if marker_pos.y<int(self.width/4):
            self.vel_y=-1.0
        if marker_pos.y>int(self.width/4) and marker_pos.y<int(self.width/4*3):
            self.vel_y=0

    def timer_callback(self):
        msg=Twist()
        temp=Vector3()
        temp.x=float(self.vel_x)
        temp.y=float(self.vel_y)
        temp.z=0.0
        msg.linear=temp
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
