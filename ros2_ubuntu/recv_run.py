'''
スタックチャンからUSBSerial経由できたプロンプトをgptに送信し、pythonファイルを生成+ビルド+実行
headerから始まり、改行\nで終わるまでをプロンプトとして受け付けている
'''
import serial
import openai
import subprocess
import os
import sys

#プロンプトと認識するためのheader
header = '!!'

# OpenAI APIキーの取得
openai.api_key = "api key"

# シリアルポートの設定（ポート名とボーレートは適宜変更してください）
SERIAL_PORT = "/dev/ttyUSB0"  # Windowsの場合。LinuxやMacの場合は'/dev/ttyUSB0'など
BAUD_RATE = 115200

# シリアルポートの初期化
ser = serial.Serial(SERIAL_PORT, BAUD_RATE)

# 事前promptをprompt.txtから読み込む
with open("prompt.txt", "r") as f:
    pre_prompt = f.read()
    # pre_prompt = ""

# ChatGPT APIでpromptを入力して返信を受け取る
def get_chat_response(prompt):
    print("a")
    response = openai.ChatCompletion.create(
        model="gpt-4o",
        messages=[
            {"role": "user", "content": prompt},
        ]
    )
    return response.choices[0]["message"]["content"].strip()

#返信の内容を.pyファイルに書いて保存する
def generate_python_script(res):
    python_code = res
    print("b")
    with open("generated_script.py", "w") as f:
        f.write(python_code)

# subprocessでpythonスクリプトを実行する
def run_python_script():
    print("c")
    result = subprocess.run(["python3", "generated_script.py"], capture_output=True)

def process_message(message):
    # 受信したメッセージに「今日の天気は」を追加
    prompt = pre_prompt + message
    print(f"prompt: {prompt}")
    res = get_chat_response(prompt)
    generate_python_script(res)
    run_python_script()


def main():
    while True:
        # print("waiting")
        if ser.in_waiting > 0:
            # シリアルポートからメッセージを読み込む
            message = ser.readline().strip().decode('UTF-8')
            if message:
                print(f"受信したメッセージ: {message}")
                if message.startswith(header):
                    processed_message = message[len(header):].strip()
                    print(f"exec_message: {processed_message}")
                    process_message(processed_message)
                    break

if __name__ == "__main__":
    main()
