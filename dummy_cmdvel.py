import rclpy
 from rclpy.node import Node
 from geometry_msgs.msg import Twist
 
 class DifferentialDriveNode(Node):
     def __init__(self):
         super().__init__('differential_drive_node')
         self.publisher_ = self.create_publisher(Twist, '/cmd_vel_mini', 10)
         self.timer = self.create_timer(0.1, self.timer_callback)
         self.get_logger().info('Differential Drive Node has been started.')
 
     def timer_callback(self):
         msg = Twist()
         msg.linear.x = 0.1  # Set linear velocity
         msg.angular.z = 0.1  # Set angular velocity
         self.publisher_.publish(msg)
         self.get_logger().info('Publishing: "%s"' % msg)
 
 def main(args=None):
     rclpy.init(args=args)
     node = DifferentialDriveNode()
     rclpy.spin(node)
     node.destroy_node()
     rclpy.shutdown()
 
 if __name__ == '__main__':
     main()
