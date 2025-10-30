import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from neato2_interfaces.msg import Bump
from enum import Enum
import tty
import select
import sys
import termios
from time import sleep
import cv2 as cv
import numpy as np

class LineFollow(Node):
    """Line following class"""

    def __init__(self):
        super().__init__("line_approach")

    def line_filter(self):
        cap = cv.VideoCapture(0)
    
        while(1):
        
            # Take each frame
            _, frame = cap.read()
        
            # Convert BGR to HSV
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
            # define range of blue color in HSV
            lower_blue = np.array([110,50,50])
            upper_blue = np.array([130,255,255])
        
            # Threshold the HSV image to get only blue colors
            mask = cv.inRange(hsv, lower_blue, upper_blue)
        
            # Bitwise-AND mask and original image
            res = cv.bitwise_and(frame,frame, mask= mask)
        
            cv.imshow('frame',frame)
            cv.imshow('mask',mask)
            cv.imshow('res',res)
            k = cv.waitKey(5) & 0xFF
            if k == 27:
                break
        
        cv.destroyAllWindows()