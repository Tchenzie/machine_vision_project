import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from time import sleep
import cv2 as cv
import numpy as np
#publish the new image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class LineFollow(Node):
    """Line following class"""

    def __init__(self):
        super().__init__("line_approach")
        self.subscription = self.create_subscription(Image, '/camera/image_raw', 
        self.image_callback,10)
        #subscribe to rqt image then publish filtered one
        self.publisher = self.create_publisher(Image, '/filtered_image_raw', 10)


    def line_filter(self,msg):
        #get image then republish
        cap = cv.VideoCapture(0)
        while(1):
        
            # Take each frame
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')        
            # Convert BGR to HSV
            hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
        
            # define range of blue color in HSV
            lower_blue = np.array([110,50,50])
            upper_blue = np.array([130,255,255])
        
            # Threshold the HSV image to get only blue colors
            mask = cv.inRange(hsv, lower_blue, upper_blue)
        
            # Bitwise-AND mask and original image
            res = cv.bitwise_and(frame,frame, mask= mask)
        
            # Convert OpenCV image to ROS image message
            ros_image = self.bridge.cv2_to_imgmsg(bw, encoding='mono8')
            # Publish the filtered image
            self.publisher.publish(ros_image)

            # Optional: Display the filtered image
            cv.imshow('Filtered Image', res)
            if cv.waitKey(1) & 0xFF == ord('q'):
                break

            cap.release()
        
        
        cv.destroyAllWindows()

    def line_detection(self):
        """ detects lines ahead"""