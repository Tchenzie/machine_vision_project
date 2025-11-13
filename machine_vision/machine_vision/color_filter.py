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
        self.mode = "SIMPLE"
        # Simple Variables.
        self.simple_left = False
        self.simple_right = False
        #Investigate Variables
        self.investigate_left = False
        self.investigate_right = False
        
        #Run the turn ONCE
        self.investigative_turn = False

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

    #STATE FUNCTIONS    
    def simple_function(self):
        """ A simple line following function. Follow the line until a maze stack is created."""

        if not self.maze_stack and not self.dead_end and not self.intersection:
                self.go_forward(0.05)
        #Checking for left or right paths but not reacting.
        if self.left_path:
                self.simple_left = True
        if self.right_path:
                self.simple_right = True

        #If dead end, then react to simple info.
        if self.dead_end and not (self.simple_left and self.simple_right):
            #Only turns if only left or right was triggered NOT both
            if self.simple_right:
                self.turn_right()
                self.simple_right = False

            if self.simple_left:
                self.turn_left
                self.simple_left = False
        #If both left and right are seen OR its an intersection. Its not simple.
        if self.intersection or (self.simple_left and self.simple_right):
            print("intersection")
            self.mode = "INTERSECTION"

    def intersection_function(self):
        """ Function that logs a found intersection"""
        #Add the stack
        if not self.maze_stack:
            self.add_intersection_stack()
        if self.maze_stack:
            self.maze_stack.append(",")
            self.add_intersection_stack()

        self.mode = "INVESTIGATE"

    def investigate_function(self):
        """ Function that turns in the direction of the top of the stack and follows the path
        until new intersection or dead end."""
        #Turn on first run only.
        if not self.investigative_turn:
            if self.maze_stack[-1] == "L":
                self.turn_left()
                self.investigative_turn = True
            if self.maze_stack[-1] == "R":
                self.turn_right()
                self.investigative_turn = True
            if self.maze_stack[-1] == "M":
                self.go_forward(0.01)
                self.investigative_turn = True
            if self.maze_stack[-1] == ",":
                self.mode = "BACK"

        #If not int or dead end
        if not self.intersection and not self.dead_end:
            self.go_forward(0.05)

        #If theres a left or right turn (not path)
        if self.left_path and not self.intersection:
                self.turn_left()
        if self.right_path and not self.intersection:
                self.turn_right()
        #If intersection or dead end switch modes.
        if self.intersection:
            self.mode = "INTERSECTION"
            self.investigative_turn = False
        if self.dead_end:
            self.mode = "BACK"
            self.investigative_turn = False

        
    def turn_back_function(self):
        """A Funtion that handles dead ends turns back 180 and prepares retrace """
        if not self.intersection:
            self.turn_right()
            self.turn_right()
        self.mode = "RETRACE"

    def retrace_steps_function(self):
        """Retraces steps until its back at an intersection"""

        
        if not self.intersection:
            self.go_forward(0.05)

        #If back to the intersection, face the original direction before turning into it.
        if self.intersection:
            #If you were investigating a sub intersection, pop the comma first
            if self.maze_stack == [","]:
                self.maze_stack.pop()
            # If you were investigating a left turn, turn left
            if self.maze_stack == ["L"]:
                self.turn_left()
            # If you were investigating a right turn, turn rigth
            if self.maze_stack == ["R"]:
                self.turn_right()
            #if you were investigating a middle path, turn 180
                self.turn_left()
                self.turn_left()
            #then pop
            sleep(0.1)
            self.maze_stack.pop()
            self.mode = "INVESTIGATE"

        #If left OR right, turn in that direction
        if self.left_trigger and not self.intersection:
            self.turn_left()
        if self.right_trigger and not self.intersection:
            self.turn_right()
        
        
    def run_loop(self):
        
        """continuously check map and move according to the state"""
        print(self.mode)
        print(self.maze_stack)
        #Simple Mode.
        if self.mode == "SIMPLE":
            self.simple_function()
        
        if self.mode == "INTERSECTION":
            self.intersection_function()

        if self.mode == "INVESTIGATE":
            self.investigate_function()
        
        if self.mode == "BACK":
            self.turn_back_function()

        if self.mode == "RETRACE":
            self.retrace_steps_function()
        
        print(self.mode)
        print(self.maze_stack)
          
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
