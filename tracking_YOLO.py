import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.object_detected = False
        self.twist = Twist()
        

        #self.image_subscription = self.create_subscription(
        #    Image,
        #    '/camera/image_raw',
        #    self.image_callback,
        #    10)

        self.obj_area=0
        self.obj_point=0
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        #subscript x,y and area, then execute cmd_vel_timer
        self.timer = self.create_timer(0.1, self.on_cmd_vel_timer)

    def on_cmd_vel_timer(self):
        LINEAR_VEL=-0.5
        ANGULAR_VEL=-0.8
        TARGET_AREA=0.1
        OBJECT_AREA_THRESHOLD=0.01

        #normailize?

        if self.object_detected and self.obj_area>OBJECT_AREA_THRESHOLD:

            # 速度コマンドを設定
            self.twist.linear.x = LINEAR_VEL*(self.obj_area-TARGET_AREA)  # 前進速度
            self.twist.angular.z = ANGULAR_VEL*self.obj_point.x  # 角速度（誤差に基づく回転）
        else:
            # 物体が検出されなかった場合、停止
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    object_tracker = ObjectTracker()

    try:
        rclpy.spin(object_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        object_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
