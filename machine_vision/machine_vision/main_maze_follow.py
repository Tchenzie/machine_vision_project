import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class maze_follow(Node):
    def __init__(self):
        super().__init__("maze_follow")
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.run = True

        self.timer = self.create_timer(0.1, self.run_loop)
    
    def forward(self, speed):
        msg = Twist()
        msg.linear.x = speed
        self.pub.publish(msg)

    def run_loop(self):
        
        """continuously check map and move accordingly"""
        if self.run:
            self.forward(0.2)
            self.run = True

    
    #def maze_navigation(self,msg):

def main(args=None):
    rclpy.init(args=args)
    node = maze_follow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()