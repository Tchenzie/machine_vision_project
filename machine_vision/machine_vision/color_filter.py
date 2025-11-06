import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from time import sleep
import cv2 as cv
import numpy as np
#publish the new image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class maze_vision(Node):
    """Color filtering class"""

    def __init__(self):

        super().__init__("color_filter")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/camera/image_raw', 
        self.image_callback,10)
        #subscribe to rqt image then publish filtered one
        self.publisher = self.create_publisher(Image, '/filtered_image_raw', 10)


    def image_callback(self,msg):
        
        #Sensors and triggers in y,x
        middle_sensor = 430,390
        left_trigger = 430,230
        right_trigger = 430,600
        middle_trigger = 300,430

        # Take each frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')        
        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
        # define range of color in HSV
        lower_color = np.array([120,50,50])
        upper_color = np.array([330,255,255])
    
        # Threshold the HSV image to get only blue colors
        mask = cv.inRange(hsv, lower_color, upper_color)
    
        # Bitwise-AND mask and original image
        res = cv.bitwise_and(frame,frame, mask= mask)
        
        # Convert OpenCV image to ROS image message
        ros_image = self.bridge.cv2_to_imgmsg(res, encoding='bgr8')
        # Publish the filtered image
        self.publisher.publish(ros_image)

        # Optional: Display the filtered image
        cv.imshow('Filtered Image', res)
        cv.waitKey(1)
        
        print(res[430,390])
        return res
        

    cv.destroyAllWindows()
    


def main(args=None):
    rclpy.init(args=args)
    node = maze_vision()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv.destroyAllWindows()
    
if __name__ == '__main__':
    main()
