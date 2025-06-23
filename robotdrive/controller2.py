import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class Controller2(Node):
    def __init__(self):
        super().__init__('controller2')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        
        self.v_max = 0.18  
        self.omega_max = math.pi / 9 

        
        self.xf = 5.17
        self.yf = 2.60
        self.thetaf = 1.722

        
        #fasi1
        
        self.theta_goal = math.atan2(self.yf, self.xf)  #0.465 rad
        self.tf1 = (3 * self.theta_goal) / (2 * self.omega_max)  
        self.a2_1 = (3 * self.theta_goal) / (self.tf1 ** 2)
        self.a3_1 = (-2 * self.theta_goal) / (self.tf1 ** 3)

        
        #fasi 2
        
        self.d = math.sqrt(self.xf**2 + self.yf**2)  
        self.tf2 = (3 * self.d) / (2 * self.v_max)
        self.a2_x = (3 * self.xf) / (self.tf2**2)
        self.a3_x = (-2 * self.xf) / (self.tf2**3)
        self.a2_y = (3 * self.yf) / (self.tf2**2)
        self.a3_y = (-2 * self.yf) / (self.tf2**3)

        
        #fasi 3
        
        self.delta_theta2 = self.thetaf - self.theta_goal
        self.tf3 = (3 * self.delta_theta2) / (2 * self.omega_max)
        self.a2_3 = (3 * self.delta_theta2) / (self.tf3 ** 2)
        self.a3_3 = (-2 * self.delta_theta2) / (self.tf3 ** 3)

  
        self.phase = 1  #ksekiname me fasi 1
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.timer_callback)  


    def cubic_velocity(self, a2, a3, t):
        return 2 * a2 * t + 3 * a3 * t**2

    def timer_callback(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9 
        twist = Twist()


        #fasi 1

        if self.phase == 1 and t <= self.tf1:
            twist.angular.z = self.cubic_velocity(self.a2_1, self.a3_1, t)

        elif self.phase == 1:
            self.phase = 2
            self.start_time = now
            return


        #fasi 2
       
        elif self.phase == 2 and t <= self.tf2:
            vx = self.cubic_velocity(self.a2_x, self.a3_x, t)
            vy = self.cubic_velocity(self.a2_y, self.a3_y, t)
            v = math.sqrt(vx**2 + vy**2)  
            twist.linear.x = v

        elif self.phase == 2:
            self.phase = 3
            self.start_time = now
            return

        #fasi 3
        
        elif self.phase == 3 and t <= self.tf3:
            twist.angular.z = self.cubic_velocity(self.a2_3, self.a3_3, t)

        elif self.phase == 3:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.get_logger().info('twist completed')
            self.destroy_timer(self.timer)
            return

        self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Controller2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

