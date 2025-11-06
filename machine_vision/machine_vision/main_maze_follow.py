import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import cv2 as cv
from time import sleep
import math

class maze_follow(Node):
    """A class executing all the maze logic"""
        
    def __init__(self):
        
        super().__init__("maze_follow")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.timer = self.create_timer(0.1, self.run_loop)
        self.subscription = self.create_subscription(
            Image,
            '/filtered_image_raw',
            self.image_callback,
            10
        )

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
        if self.bump_state == False:
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
    
        

    def run_loop(self,msg):
        maze_stack = []
        middle_sensor = 430,390
        left_trigger = 430,230
        right_trigger = 430,600
        middle_trigger = 300,430

        left_path = res[left_trigger][2]!=0
        middle_path = res[middle_trigger][2]!=0
        right_path = res[right_trigger][2]!=0

        centered_on_tape = res[middle_sensor][2]!=0

        

        res = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        intersection = (res[middle_trigger][2]!=0)+(res[left_trigger][2]!=0+(res[right_trigger][2]!=0) )>1
        dead_end = res[middle_trigger][2]=0 and res[left_trigger][2]=0 and res[right_trigger][2]=0 
        """continuously check map and move accordingly"""

        if not maze_stack:
            #If any 2 directions are sensed at once
            #Sensed: if red values are greater than 100. 0=blue,1=green, 2=red
            if (intersection):
                #Add all the sensed directions to the maze stack.
                if (left_path):
                    maze_stack.append("L")
                if (middle_path):
                    maze_stack.append("M")
                if (right_path):
                    maze_stack.append("R")
            
            #If Left or Right Path is seen  
            elif (left_path or right_path) and not intersection:
                #Move forward for some distance before turning
                if right_path:
                    self.go_forward(1)
                    self.turn_right
                    
                if left_path:
                    self.go_forward(1)
                    self.turn_left

            #If centered on tape go forward
            elif centered_on_tape and not intersection:
                self.forward(0.2)
        #If there's a maze stack begin investigations
        if maze_stack:  
            if not dead_end:
                if maze_stack:
                    if maze_stack.top("L"):
                        self.turn_left()
                    if maze_stack.top("R"):
                        self.turn_right()
                    while not intersection:
                        self.go_forward(1)
        

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