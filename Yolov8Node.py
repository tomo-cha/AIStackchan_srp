import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO
import numpy as np
import cv2

class Yolov8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')

        # YOLOv8モデルの読み込み
        self.model = YOLO('yolov8m.pt')  # 適切なモデルをロード
        self.model.fuse()

        # CvBridgeのインスタンスを作成してROS 2画像メッセージをOpenCV形式に変換
        self.bridge = CvBridge()

        # サブスクリプションの作成
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.image_callback,
            10)

        #オブジェクト検出
        self.object_detected = False
        self.object_normalized_area=0
        self.object_normalized_point_x=0

        #twist
        self.twist = Twist()
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.on_cmd_vel_timer)
        

    def image_callback(self, msg: Image):
        # 画像メッセージをOpenCV形式に変換
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        #画像のフルサイズを獲得
        full_height,full_width=cv_image.shape[:2]

        # YOLOv8で物体検出を実行
        results = self.model(cv_image)

        # 検出結果を処理して、バウンディングボックスと重心を描画
        for result in results:
            for box in result.boxes:
                # バウンディングボックスの座標（中心 x, y, 幅, 高さ）
                x_center = float(box.xywh[0][0])
                y_center = float(box.xywh[0][1])
                width = float(box.xywh[0][2])
                height = float(box.xywh[0][3])

                #箱の面積
                obj_area=width*height
                
                #面積とx座標に対して正規化処理
                self.object_normalized_area = obj_area/(full_height*full_width)
                self.object_normalized_point_x = 2.0*x_center/full_width-1.0

                # クラスID、クラス名、信頼度
                class_id = int(box.cls)
                class_name = self.model.names[int(box.cls)]
                confidence = float(box.conf)

                # コンソールに表示
                print(f"Class: {class_name} (ID: {class_id}), Confidence: {confidence:.2f}")
                print(f"BoundingBox - X: {x_center:.2f}, Y: {y_center:.2f}, Width: {width:.2f}, Height: {height:.2f}")
                print("-" * 50)

                # バウンディングボックスを描画
                x_min = int(x_center - width / 2)
                y_min = int(y_center - height / 2)
                x_max = int(x_center + width / 2)
                y_max = int(y_center + height / 2)

                # バウンディングボックスの四角形を描画
                cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                # 重心を描画
                cv2.circle(cv_image, (int(x_center), int(y_center)), 5, (0, 0, 255), -1)

                # クラス名と信頼度を画像上に描画
                label = f"{class_name} ({confidence:.2f})"
                cv2.putText(cv_image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        # 検出結果をウィンドウに表示
        cv2.imshow("YOLOv8 Detection", cv_image)

        # ウィンドウが閉じられるまでキー入力を待つ
        cv2.waitKey(1)

    def on_cmd_vel_timer(self):
        #速度−閾値など変数
        LINEAR_VEL=-0.5
        ANGULAR_VEL=-0.8
        TARGET_AREA=0.1
        OBJECT_AREA_THRESHOLD=0.01
        

        if self.object_detected and self.object_normalized_area>OBJECT_AREA_THRESHOLD:
            # 速度コマンドを設定
            self.twist.linear.x = LINEAR_VEL*(self.object_normalized_area-TARGET_AREA)  # 前進速度
            self.twist.angular.z = ANGULAR_VEL*self.object_normalized_point_x # 角速度（誤差に基づく回転）
        else:
            # 物体が検出されなかった場合、停止
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
        
        self.cmd_vel_publisher.publish(self.twist)
    
def main(args=None):
    rclpy.init(args=args)
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
