import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt, sin, cos

class Controller3(Node):
    def __init__(self):
        super().__init__('Controller3')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        #stoxos 
        self.xd = 5.17
        self.yd = 2.6
        self.thetad = 1.722 


        self.kr = 0.4 #katefthinsi apo to robot ston stoxo
        self.ka = 1.5 #katefthinsi pros ton stoxo(gwnia)
        self.kb = -0.6 #sfalma gwniakou prosanatolismou

       
        self.v_max = 0.18
        self.w_max = 0.349

    def odom_callback(self, msg):   #ekteleite kathe fora pou allazei thesi to robot
        #pos
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        #metatropi se theta  
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        theta = atan2(siny_cosp, cosy_cosp)

        #ipologizo metavlites elegxou
        dx = self.xd - x
        dy = self.yd - y
        r = sqrt(dx**2 + dy**2)
        a = atan2(dy, dx) - theta
        b = -theta - a + self.thetad

        #kanonikopoiisi gwniwn
        a = atan2(sin(a), cos(a))
        b = atan2(sin(b), cos(b))

        #eksodos elegkti
        v = self.kr * r
        w = self.ka * a + self.kb * b
        #periorizo taxitites
        v = max(-self.v_max, min(self.v_max, v))
        w = max(-self.w_max, min(self.w_max, w))

        #elegxw an exei oloklirothei
        if r < 0.05 and abs(self.thetad - theta) < 0.05:
            self.get_logger().info('egine')
            self.cmd_pub.publish(Twist())
            return

        #kanei publish tin taxitita
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = w
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = Controller3()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

