import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2 as cv
from time import sleep
import math
from cv_bridge import CvBridge

class maze_follow(Node):
    """A class executing all the maze logic"""
        
    def __init__(self):
        
        super().__init__("maze_follow")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.run_loop)
        self.subscription = self.create_subscription(
            Image,
            '/filtered_image_raw',
            self.line_follower,
            10
        )
        self.maze_stack = []
        self.left_path = False
        self.middle_path = False
        self.right_path = False
        self.intersection = False
        self.dead_end = False
        self.centered_on_tape = False
        self.latest_image = None



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
    
    def line_follower_callback(self, msg):
    # Convert ROS Image → OpenCV
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        res = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
    def line_follower(self):
        
        if self.latest_image is None:
         return  # no image yet

        res = self.latest_image
        self.maze_stack = []
        self.middle_sensor = 430,390
        self.left_trigger = 430,230
        self.right_trigger = 430,600
        self.middle_trigger = 300,430

        self.left_path = res[self.left_trigger][2]!=0
        self.middle_path = res[self.middle_trigger][2]!=0
        self.right_path = res[self.right_trigger][2]!=0

        self.centered_on_tape = res[self.middle_sensor][2]!=0

        self.intersection = (
        (res[self.middle_trigger][2]!=0)+
        (res[self.left_trigger][2]!=0) +
        (res[self.right_trigger][2]!=0) 
        )>1
        self.dead_end = res[self.middle_trigger][2]==0 and (res[self.left_trigger][2])==0 and res[self.right_trigger][2]==0


    def add_intersection_stack(self):
        if (self.left_path):
            self.maze_stack.append("L")
        if (self.middle_path):
            self.maze_stack.append("M")
        if (self.right_path):
            self.maze_stack.append("R")


    #If Left or Right Path is seen  
        if (self.left_path or self.right_path) and not self.intersection:
            #Move forward for some distance before turning
            if self.right_path:
                self.go_forward(1)
                self.turn_right()
                
            if self.left_path:
                self.go_forward(1)
                self.turn_left()

        #If centered on tape go forward
        elif self.centered_on_tape and not self.intersection:
            self.go_forward(0.2)
        

    def run_loop(self):
    
        """continuously check map and move accordingly"""
        
        if not self.maze_stack:
            #If any 2 directions are sensed at once
            #Sensed: if red values are greater than 100. 0=blue,1=green, 2=red
            
            if (self.intersection):
                #Add all the sensed directions to the maze stack.
                self.add_intersection_stack()
            
            #else line follow
            else:
                self.line_follower()
        #If there's a maze stack begin investigations
        if self.maze_stack:  
            if not self.dead_end:
                if self.maze_stack:
                    if self.maze_stack[-1] == "L":
                        self.turn_left()
                    if self.maze_stack[-1] == "R":
                        self.turn_right()
                    while not self.intersection:
                        self.line_follower()

                    if self.intersection:
                        self.maze_stack.append(",")
                        self.add_intersection_stack()

            if self.dead_end and self.maze_stack[-1]!=",":
                if self.maze_stack[-1] == "L":
                    self.turn_left()
                    self.turn_left()
                    while not self.intersection:
                        self.line_follower()
                    self.turn_left()
                    self.maze_stack.pop()
                elif self.maze_stack[-1] == "R":
                    self.turn_right()
                    self.turn_right()
                    while not self.intersection:
                        self.line_follower()
                    self.turn_right()
                    self.maze_stack.pop()
                elif self.maze_stack[-1] == "M":
                    self.turn_left()
                    self.turn_left()
                    while not self.intersection:
                        self.line_follower()
                    self.turn_left()
                    self.turn_left()
                    self.maze_stack.pop()
            if self.dead_end and self.maze_stack[-1]==",":
                self.turn_right()
                self.turn_right()
                while not self.intersection:
                        self.line_follower()
                self.maze_stack.pop()
                if self.maze_stack[-1] == "R":
                    self.turn_left()
                    self.turn_left()
                elif self.maze_stack[-1] == "L":
                    self.turn_right()
                    self.turn_right()
                self.maze_stack.pop()        
        

        # get res as a normal numpy OpenCV image
        #cv.imshow('Received Filtered Image', res)
        #cv.waitKey(1)

    
    #def maze_navigation(self,msg):

def main(args=None):
    rclpy.init(args=args)
    node = maze_follow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()