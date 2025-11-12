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
        self.following_path = False
        self.line_stack = [] #A stack for line follower

    def image_callback(self,msg):

        # FIRST FILTER THE IMAGE

        # Take each frame
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')        
        # Convert BGR to HSV
        hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    
        # define range of color in HSV
        lower_color = np.array([20,100,100])
        upper_color = np.array([35,255,255])
    
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
        #x axis in image ranges from 0 to 757
        
        self.left_trigger = [(y, x) for y in range(430, 400, -1) for x in range(0, 150)]
        self.middle_sensor = [(y, x) for y in range(430, 400, -1) for x in range(150,607)]
        self.right_trigger = [(y, x) for y in range(430, 400, -1) for x in range(607,757)]
        self.middle_trigger = [(y, x) for y in range(250, 200, -1) for x in range(150,607)]
        
        #check image for if neato is on tape if at least 3 are in the box
        self.centered_on_tape = (sum(res[point][1] > 100 for point in self.middle_sensor) /3 )>1
                                
        #or res[self.middle_sensor[0]][2]>100

        #check image for what paths neato sees
        self.left_path = (sum(res[point][1] > 100 for point in self.left_trigger) /3 )>1
        self.middle_path = ((sum(res[point][1] > 100 for point in self.middle_trigger) /10 )>1) and (self.left_path or self.right_path)
        #middle path is more strict so it is not confused by centered on tape.
        self.right_path = (sum(res[point][1] > 100 for point in self.right_trigger) /3 )>1

        #If Neato sees multiple paths it is at an intersection
        self.intersection = ((self.middle_path and self.left_path) or (self.middle_path and self.right_path) or (self.right_path and self.left_path))
        #Dead end if no paths detected 
        self.dead_end = (not self.middle_path) and (not self.left_path) and (not self.right_path) and not self.centered_on_tape
    
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

    #this is different from go forwad because it is constantly checking the image.
    #not just going forward by a set distance
    #Does not work yet
    def follow_forward_path(self):
        """ Drives Neato continuously in straight line until the neato detects a new path """
        forward_vel = 0.0
        
        while self.centered_on_tape and not self.left_path and not self.right_path:
            self.drive(linear=forward_vel, angular=0.0)
            rclpy.spin_once #check the img
            sleep(0.1)
            
        self.drive(linear=0.0, angular=0.0)
        

    def turn_right(self):
        """Turns Neato 90 degrees based on set angular velocity """
        self.go_forward(0.35) #forward by buffer before turning
        ang_vel = 0.2
        self.drive(linear= 0.0, angular= -ang_vel)
        sleep(((math.pi/ang_vel)*(1/2)))
        self.drive(linear=0.0, angular=0.0)

    def turn_left(self):
        """Turns Neato 90 degrees based on set angular velocity """
        self.go_forward(0.35) #Go forward by buffer before turning
        ang_vel = 0.2
        self.drive(linear= 0.0, angular= ang_vel)
        sleep(((math.pi/ang_vel)*(1/2)))
        self.drive(linear=0.0, angular=0.0)
    
    
    def line_follower(self):
        if self.left_path and not self.intersection:
            self.line_stack.append("L")
        if self.right_path and not self.intersection:
            self.line_stack.append("R")
        if self.dead_end and not self.intersection and self.line_stack:
            if self.line_stack[-1] == "L":
                self.turn_left()
            if self.line_stack[-1] == "R":
                self.turn_right()

        #If centered on tape go forward
        elif self.centered_on_tape and not self.intersection and not self.dead_end:
            self.go_forward(0.05)
            print("centered")

    def add_intersection_stack(self):
        if (self.left_path):
            self.maze_stack.append("L")
        if (self.middle_path):
            self.maze_stack.append("M")
        if (self.right_path):
            self.maze_stack.append("R")
    
    #A function for testing.
    def stop(self):
        "Make the Neato Stop"
        self.drive(linear=0.0, angular=0.0)

    

    def run_loop(self):
        
        """continuously check map and move accordingly"""
        #NTS: Will this run once? !maze stack...
        #If any 2 directions are sensed at once
        if not self.maze_stack:
            if self.intersection:
                self.add_intersection_stack()
            else:
                print(self.right_path)
                print(self.left_path)
                print(self.intersection)
                self.line_follower()
        print(self.maze_stack)

        if self.maze_stack: 
             print(self.maze_stack) 
             if not self.dead_end:
                 sleep(3)
                 if self.maze_stack:
                     print(self.maze_stack[-1])
                     if self.maze_stack[-1] == "L":
                        self.turn_left()
                     if self.maze_stack[-1] == "R":
                        self.turn_right()
                     sleep(0.1)
                     print(self.intersection)
                     if not self.intersection:
                         self.line_follower()
                         self.following_path = True
                         print("following path is true")
                         if self.intersection or self.dead_end:
                             self.following_path = False

                     if self.intersection and not self.following_path:
                         self.maze_stack.append(",")
                         self.add_intersection_stack()
                         print(self.maze_stack)
             if self.dead_end and self.maze_stack[-1]!="," and not self.following_path:
                if self.maze_stack[-1] == "L":
                    self.turn_left()
                    self.turn_left()
                    if not self.intersection:
                       self.line_follower()
                    self.turn_left()
                    self.maze_stack.pop()
                elif self.maze_stack[-1] == "R":
                    self.turn_right()
                    self.turn_right()
                    if not self.intersection:
                        self.line_follower()
                    self.turn_right()
                    self.maze_stack.pop()
                elif self.maze_stack[-1] == "M":
                    self.turn_left()
                    self.turn_left()
                    if not self.intersection:
                        self.line_follower()
                    self.turn_left()
                    self.turn_left()
                    self.maze_stack.pop()
             if self.dead_end and self.maze_stack[-1]=="," and not self.following_path:
                self.turn_right()
                self.turn_right()
                if not self.intersection:
                        self.line_follower()
                self.maze_stack.pop()
                if self.maze_stack[-1] == "R":
                    self.turn_left()
                    self.turn_left()
                elif self.maze_stack[-1] == "L":
                    self.turn_right()
                    self.turn_right()
                self.maze_stack.pop()

             if self.following_path and not (self.intersection or self.dead_end):
                 self.line_follower()
                 print("Following line")
                


        
        
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
        #    

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
