import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from time import sleep
import cv2 as cv
import numpy as np
#publish the new image
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import math 

class maze_vision(Node):
    """Color filtering class"""

    def __init__(self):

        super().__init__("color_filter")
        
        self.bridge = CvBridge()

        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.run_loop)

        self.subscription = self.create_subscription(Image, '/camera/image_raw', 
        self.image_callback,10)
        #subscribe to rqt image then publish filtered one
        self.publisher = self.create_publisher(Image, '/filtered_image_raw', 10)
        self.maze_stack = []
        self.left_path = False
        self.middle_path = False
        self.right_path = False
        self.intersection = False
        self.dead_end = False
        self.centered_on_tape = False
        self.latest_image = None

    def image_callback(self,msg):

        # FIRST FILTER THE IMAGE
        
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
        lower_color = np.array([120,0,0])
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
        
        
        #CHANGE THE VARIABLES BASED ON THE IMAGE

        #Define points
        self.middle_sensor = 430,390
        self.left_trigger = 430,230
        self.right_trigger = 430,600
        self.middle_trigger = 300,430

        #check image for if neato is on tape
        self.centered_on_tape = res[self.middle_sensor][2]>100

        #check image for what paths neato sees
        self.left_path = res[self.left_trigger][2]>100
        self.middle_path = res[self.middle_trigger][2]>100
        self.right_path = res[self.right_trigger][2]>100

        #If Neato sees multiple paths it is at an intersection
        self.intersection = (self.middle_path + self.left_path + self.right_path)>1
        #Dead end if no paths detected 
        self.dead_end = (not self.middle_path) & (not self.left_path) & (not self.right_path)
    
    def drive(self, linear, angular):
        """ Publishes cmd_vel to Twist topic to set Neato linear and angular velocities
        Args: 
            linear: linear velocity in m/s
            angular: angular velocity in rad/s
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub.publish(msg)

    def go_forward(self, distance):
        """ Drives Neato in straight line based on specified velocity
        Args:
            distance: specified distance (meters)
        
        """
        forward_vel = 0.1
        
        self.drive(linear=forward_vel, angular=0.0)
        sleep(distance/forward_vel)
        self.drive(linear=0.0, angular=0.0)

    def turn_right(self):
        """Turns Neato 90 degrees based on set angular velocity """
        ang_vel = 0.2
        self.drive(linear= 0.0, angular= -ang_vel)
        sleep(((math.pi/ang_vel)*(1/2)))
        self.drive(linear=0.0, angular=0.0)

    def turn_left(self):
        """Turns Neato 90 degrees based on set angular velocity """
        ang_vel = 0.2
        self.drive(linear= 0.0, angular= ang_vel)
        sleep(((math.pi/ang_vel)*(1/2)))
        self.drive(linear=0.0, angular=0.0)
    
    
    def line_follower(self):

        self.maze_stack = []
        print(self.centered_on_tape)

        #If Left or Right Path is seen  
        if (self.left_path or self.right_path) and not self.intersection:
            #Move forward for some distance before turning
            if self.right_path:
                self.go_forward(0.2)
                self.turn_right(0.2)
                
            if self.left_path:
                self.go_forward(0.2)
                self.turn_left(0.2)

        #If centered on tape go forward
        elif self.centered_on_tape and not self.intersection:
            self.go_forward(0.2)

    def add_intersection_stack(self):
        if (self.left_path):
            self.maze_stack.append("L")
        if (self.middle_path):
            self.maze_stack.append("M")
        if (self.right_path):
            self.maze_stack.append("R")


    

    def run_loop(self):
        """continuously check map and move accordingly"""
        self.line_follower()

        # print(self.centered_on_tape)
        # if self.centered_on_tape:
        #     self.go_forward(0.2)
        # print(self.line_follower())
        # if not self.maze_stack:
        #     #If any 2 directions are sensed at once
        #     #Sensed: if red values are greater than 100. 0=blue,1=green, 2=red
            
        #     if (self.intersection):
        #         #Add all the sensed directions to the maze stack.
        #         self.add_intersection_stack()
            
        #     #else line follow
        #     else:
        #         self.line_follower()
        # #If there's a maze stack begin investigations
        # if self.maze_stack:  
        #     if not self.dead_end:
        #         if self.maze_stack:
        #             if self.maze_stack[-1] == "L":
        #                 self.turn_left()
        #             if self.maze_stack[-1] == "R":
        #                 self.turn_right()
        #             while not self.intersection:
        #                 self.line_follower()

        #             if self.intersection:
        #                 self.maze_stack.append(",")
        #                 self.add_intersection_stack()

        #     if self.dead_end and self.maze_stack[-1]!=",":
        #         if self.maze_stack[-1] == "L":
        #             self.turn_left()
        #             self.turn_left()
        #             while not self.intersection:
        #                 self.line_follower()
        #             self.turn_left()
        #             self.maze_stack.pop()
        #         elif self.maze_stack[-1] == "R":
        #             self.turn_right()
        #             self.turn_right()
        #             while not self.intersection:
        #                 self.line_follower()
        #             self.turn_right()
        #             self.maze_stack.pop()
        #         elif self.maze_stack[-1] == "M":
        #             self.turn_left()
        #             self.turn_left()
        #             while not self.intersection:
        #                 self.line_follower()
        #             self.turn_left()
        #             self.turn_left()
        #             self.maze_stack.pop()
        #     if self.dead_end and self.maze_stack[-1]==",":
        #         self.turn_right()
        #         self.turn_right()
        #         while not self.intersection:
        #                 self.line_follower()
        #         self.maze_stack.pop()
        #         if self.maze_stack[-1] == "R":
        #             self.turn_left()
        #             self.turn_left()
        #         elif self.maze_stack[-1] == "L":
        #             self.turn_right()
        #             self.turn_right()
        #         self.maze_stack.pop()   

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
