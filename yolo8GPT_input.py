import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import torch
from ultralytics import YOLO
import numpy as np
import cv2
import openai
import serial
from sensor_msgs.msg import CompressedImage

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
            CompressedImage,
            'image_raw/compressed', 
            self.image_callback,
            10)

        
        # twistメッセージ
        self.twist = Twist()
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.on_cmd_vel_timer)

        self.detected_objects = []  # 検出されたオブジェクトのリスト
        self.selected_object_class = None  # 選択されたオブジェクトのクラス名

        # シリアルポートの設定（利用可能な場合のみ）
        try:
            self.serial_port = serial.Serial("/dev/ttyUSB0", 115200)
            self.use_serial = True
        except serial.SerialException:
            print("シリアルポートが利用できないため、コンソール入力に切り替えます。")
            self.serial_port = None  # シリアルポートが利用できない場合はNoneに設定
            self.use_serial = False

        # OpenAI APIキーの設定
        openai.api_key = "api-key"
    

    def image_callback(self, msg: CompressedImage):
        # 画像メッセージをOpenCV形式に変換
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

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

                # 検出されたオブジェクトをリストに追加
                self.detected_objects.append({
                    "class_name": class_name, 
                    "class_id": class_id,
                    # "x_min": int(x_center - width / 2),
                    # "y_min": int(y_center - height / 2),
                    # "x_max": int(x_center + width / 2),
                    # "y_max": int(x_center + width / 2)
                })

        # 検出結果をウィンドウに表示
        cv2.imshow("YOLOv8 Detection", cv_image)
        # print(self.detected_objects)

        # ウィンドウが閉じられるまでキー入力を待つ
        cv2.waitKey(1)

    def ask_user_for_input(self):
        # 検出されたオブジェクトについて質問
        question = "何に使用するものが欲しいですか？"
        print(question)

        # シリアル通信で入力待ち（シリアルポートが利用できる場合）
        if self.serial_port and self.serial_port.in_waiting > 0:
            user_input = self.serial_port.readline().decode('utf-8').strip()
            print(f"ユーザー入力: {user_input}")
            return user_input
        else:
            # コンソールからの入力を利用
            user_input = input("入力してください: ")
            print(f"ユーザー入力 (コンソール): {user_input}")
            return user_input


    def get_gpt_response(self, user_input):
        #検出された物体リストを文字列としてフォーマット
        object_list = ", ".join([f"{obj['class_name']} (ID: {obj['class_id']})" for obj in self.detected_objects])
        prompt = f"検出された物体のリスト: {object_list}\n\n{user_input}に適した物体を1tudake選んでください。"

        response = openai.ChatCompletion.create(
            model="gpt-4",  # 使用するモデルを指定
            messages=[  
                {"role": "user", "content": prompt},
            ]
        )  

        # 正しいキー 'message' 内の 'content' を取得
        return response.choices[0]['message']['content'].strip()
        # return "person"

    def on_cmd_vel_timer(self):
        # 物体が選択される前かどうかを確認
        if self.selected_object_class is None:
            user_input = self.ask_user_for_input()  # 1回目のユーザー入力
            if user_input:
                gpt_response = self.get_gpt_response(user_input)
                print(f"GPTの選択: {gpt_response}")

                # GPTが選択したオブジェクトのクラスを保存
                for obj in self.detected_objects:
                    if obj['class_name'] in gpt_response:
                        self.selected_object_class = obj['class_name']
                        break



def main(args=None):
    rclpy.init(args=args)
    node = Yolov8Node()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()