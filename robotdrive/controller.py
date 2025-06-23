import rclpy                          
from rclpy.node import Node           
from geometry_msgs.msg import Twist   
import time                           
import math                           

class Controller(Node):
    def __init__(self):
        super().__init__('controller') 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  
        self.start_time = time.time()   
        self.phase = 1              
        self.timer = self.create_timer(0.1, self.move_robot)  

    def move_robot(self):  
        elapsed = time.time() - self.start_time
        msg = Twist()

        if self.phase == 1:
            if elapsed < 15:
                msg.linear.x = 0.0
                msg.angular.z = math.radians(12)
            else:
                self.phase = 2
                self.start_time = time.time()
                self.get_logger().info("fasi 2")

        elif self.phase == 2:
            if elapsed < 20:
                msg.linear.x = 0.15
                msg.angular.z = math.radians(-8)
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.publisher_.publish(msg)  
                self.get_logger().info("i kinisi oloklirothike")
                rclpy.shutdown()
                return  

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)         
    node = Controller()     
    rclpy.spin(node)                
    node.destroy_node()            
    rclpy.shutdown()

