#include <Arduino.h>
//#include <FS.h>
#include <SD.h>
#include <SPIFFS.h>
#include <M5Unified.h>
#include <nvs.h>
#include <Avatar.h>

#include <AudioOutput.h>
#include <AudioFileSourceBuffer.h>
#include <AudioGeneratorMP3.h>
#include "AudioFileSourceHTTPSStream.h"
#include "AudioOutputM5Speaker.h"
#include <ServoEasing.hpp> // https://github.com/ArminJo/ServoEasing       
#include "WebVoiceVoxTTS.h"

#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "rootCACertificate.h"
#include "rootCAgoogle.h"
#include <ArduinoJson.h>
#include <ESP32WebServer.h>
#define ENABLE_MDNS
#ifdef ENABLE_MDNS
#define MDNS_HOST "stack-chan"
#include <ESPmDNS.h>
#endif
#include <deque>
#include "AudioWhisper.h"
#include "Whisper.h"
#include "Audio.h"
#include "CloudSpeechClient.h"
#include "WakeWord.h"

//ファイル作るのに必要かも
#include <fstream>
#include <iostream>

// 保存する質問と回答の最大数
const int MAX_HISTORY = 5;

// 過去の質問と回答を保存するデータ構造
std::deque<String> chatHistory;

#define USE_SDCARD
#define WIFI_SSID "SET YOUR WIFI SSID"
#define WIFI_PASS "SET YOUR WIFI PASS"
#define OPENAI_APIKEY "SET YOUR OPENAI APIKEY"
#define VOICEVOX_APIKEY "SET YOUR VOICEVOX APIKEY"
#define STT_APIKEY "SET YOUR STT APIKEY"

#define USE_SERVO
#ifdef USE_SERVO
#if defined(ARDUINO_M5STACK_Core2)
#define PORT_A
//#define PORT_C
#if defined (PORT_A)
  #define SERVO_PIN_X 33  //Core2 PORT A
  #define SERVO_PIN_Y 32
#elif defined (PORT_C)
  #define SERVO_PIN_X 14  //Core2 PORT C (INTERNAL UART2)
  #define SERVO_PIN_Y 13
#endif
#elif defined( ARDUINO_M5STACK_FIRE )
  #define SERVO_PIN_X 21
  #define SERVO_PIN_Y 22
#elif defined( ARDUINO_M5Stack_Core_ESP32 )
  #define SERVO_PIN_X 21
  #define SERVO_PIN_Y 22
#elif defined( ARDUINO_M5STACK_CORES3 )
  #define SERVO_PIN_X 18  //CoreS3 PORT C
  #define SERVO_PIN_Y 17
#endif
#endif


// --------------------         2024-08-16
// LEDの ON/OFF スイッチ
#define USE_LED                 // ｽﾀｯｸﾁｬﾝでLEDを利用する場合はコメントアウト (内蔵LED、NECOMIMI_LEDのどちらか片方だけ使う場合にもコメントアウト必須)
#define USE_INTERNAL_LED        // 内蔵LEDを使う場合はコメントアウト
#define USE_NECOMIMI_LED        // NECOMIMI_LEDを使う場合はコメントアウト
// #define USE_MEMCHECK            // ヒープメモリの監視を行う場合はコメントアウト


// --------------------         2024-08-16
// LED関連ライブラリの読み込み
#ifdef USE_LED
  #include <esp_task_wdt.h>     // ウォッチドッグタイマーのチェック
  #include <esp_heap_caps.h>    // メモリ監視 2024-08-16
  #define FASTLED_ALLOW_INTERRUPTS 0
  #include <FastLED.h>          // FastLED ライブラリ
  #include "NeoPixelEffects.h"  // LEDエフェクト ライブラリ

// --------------------         2024-08-16
// 内蔵LEDの初期エフェクト(レインボー)
#ifdef USE_INTERNAL_LED

  #define INTERNAL_PIN 25
  #define INTERNAL_LEDS 10

  int current_INT_EffectIndex = 0;
  CRGB internal_leds[INTERNAL_LEDS];
 
  NeoPixelEffects effects01 = NeoPixelEffects(
    internal_leds,
    RAINBOWWAVE,    // エフェクトの種類      effect
    0,              // エフェクト開始位置    pixstart
    9,              // エフェクト終了位置    pixend
    3,              // 点灯範囲 (COMET等)    aoe
    75,             // エフェクトの間隔      delay_ms
    CRGB::Green,    // 色 (FastLED 指定色)   color_crgb
    true,           // ループするかどうか？  looping
    FORWARD         // エフェクトの方向      direction
  );
#endif

// --------------------         2024-08-16
// NECOMIMI_LEDの初期エフェクト(レインボー)

#ifdef USE_NECOMIMI_LED

  #define PORT_B_PIN 26
  #define PORT_B_LEDS 18      // LED弾数と仕様により可変させてください

  int current_NECO_EffectIndex = 0;
  CRGB pb_leds[PORT_B_LEDS];

  NeoPixelEffects effects02 = NeoPixelEffects(
    pb_leds,
    RAINBOWWAVE,    // エフェクトの種類      effect
    0,              // エフェクト開始位置    pixstart
    17,             // エフェクト終了位置    pixend
    5,              // 点灯範囲 (COMET等)    aoe
    75,             // エフェクトの間隔      delay_ms
    CRGB::Green,    // 色 (FastLED 指定色)   color_crgb
    true,           // ループするかどうか？  looping
    FORWARD         // エフェクトの方向      direction
  );
#endif

#endif


/// set M5Speaker virtual channel (0-7)
static constexpr uint8_t m5spk_virtual_channel = 0;

using namespace m5avatar;
Avatar avatar;
const Expression expressions_table[] = {
  Expression::Neutral,
  Expression::Happy,
  Expression::Sleepy,
  Expression::Doubt,
  Expression::Sad,
  Expression::Angry
};

ESP32WebServer server(80);
//prompt
String pre_prompt;
//---------------------------------------------
String OPENAI_API_KEY = "";
String VOICEVOX_API_KEY = "";
String STT_API_KEY = "";
String TTS_SPEAKER_NO = "3";
String TTS_SPEAKER = "&speaker=";
String TTS_PARMS = TTS_SPEAKER + TTS_SPEAKER_NO;
//---------------------------------------------
bool wakeword_is_enable = false;
//---------------------------------------------
// C++11 multiline string constants are neato...
static const char HEAD[] PROGMEM = R"KEWL(
<!DOCTYPE html>
<html lang="ja">
<head>
  <meta charset="UTF-8">
  <title>AIｽﾀｯｸﾁｬﾝ</title>
</head>)KEWL";

static const char APIKEY_HTML[] PROGMEM = R"KEWL(
<!DOCTYPE html>
<html>
  <head>
    <meta charset="UTF-8">
    <title>APIキー設定</title>
  </head>
  <body>
    <h1>APIキー設定</h1>
    <form>
      <label for="role1">OpenAI API Key</label>
      <input type="text" id="openai" name="openai" oninput="adjustSize(this)"><br>
      <label for="role2">VoiceVox API Key</label>
      <input type="text" id="voicevox" name="voicevox" oninput="adjustSize(this)"><br>
      <label for="role3">Speech to Text API Key</label>
      <input type="text" id="sttapikey" name="sttapikey" oninput="adjustSize(this)"><br>
      <button type="button" onclick="sendData()">送信する</button>
    </form>
    <script>
      function adjustSize(input) {
        input.style.width = ((input.value.length + 1) * 8) + 'px';
      }
      function sendData() {
        // FormDataオブジェクトを作成
        const formData = new FormData();

        // 各ロールの値をFormDataオブジェクトに追加
        const openaiValue = document.getElementById("openai").value;
        if (openaiValue !== "") formData.append("openai", openaiValue);

        const voicevoxValue = document.getElementById("voicevox").value;
        if (voicevoxValue !== "") formData.append("voicevox", voicevoxValue);

        const sttapikeyValue = document.getElementById("sttapikey").value;
        if (sttapikeyValue !== "") formData.append("sttapikey", sttapikeyValue);

	    // POSTリクエストを送信
	    const xhr = new XMLHttpRequest();
	    xhr.open("POST", "/apikey_set");
	    xhr.onload = function() {
	      if (xhr.status === 200) {
	        alert("データを送信しました！");
	      } else {
	        alert("送信に失敗しました。");
	      }
	    };
	    xhr.send(formData);
	  }
	</script>
  </body>
</html>)KEWL";

static const char ROLE_HTML[] PROGMEM = R"KEWL(
<!DOCTYPE html>
<html>
<head>
	<title>ロール設定</title>
	<meta charset="UTF-8">
	<meta name="viewport" content="width=device-width, initial-scale=1.0">
	<style>
		textarea {
			width: 80%;
			height: 200px;
			resize: both;
		}
	</style>
</head>
<body>
	<h1>ロール設定</h1>
	<form onsubmit="postData(event)">
		<label for="textarea">ここにロールを記述してください。:</label><br>
		<textarea id="textarea" name="textarea"></textarea><br><br>
		<input type="submit" value="Submit">
	</form>
	<script>
		function postData(event) {
			event.preventDefault();
			const textAreaContent = document.getElementById("textarea").value.trim();
//			if (textAreaContent.length > 0) {
				const xhr = new XMLHttpRequest();
				xhr.open("POST", "/role_set", true);
				xhr.setRequestHeader("Content-Type", "text/plain;charset=UTF-8");
			// xhr.onload = () => {
			// 	location.reload(); // 送信後にページをリロード
			// };
			xhr.onload = () => {
				document.open();
				document.write(xhr.responseText);
				document.close();
			};
				xhr.send(textAreaContent);
//        document.getElementById("textarea").value = "";
				alert("Data sent successfully!");
//			} else {
//				alert("Please enter some text before submitting.");
//			}
		}
	</script>
</body>
</html>)KEWL";

String speech_text = "";
String speech_text_buffer = "";
DynamicJsonDocument chat_doc(1024*10);
String json_ChatString = "{\"model\": \"gpt-4o-mini\",\"messages\": [{\"role\": \"user\", \"content\": \"""\"}]}";
String Role_JSON = "";

bool Now_mode = false;

bool init_chat_doc(const char *data)
{
  DeserializationError error = deserializeJson(chat_doc, data);
  if (error) {
    //Serial.println("DeserializationError");
    return false;
  }
  String json_str; //= JSON.stringify(chat_doc);
  serializeJsonPretty(chat_doc, json_str);  // 文字列をシリアルポートに出力する
//  Serial.println(json_str);
    return true;
}


// --------------------         2024-08-16
// AIｽﾀｯｸﾁｬﾝ2で利用するエフェクトの種類 (現在は11種を用意 ※最後は消灯)
#ifdef USE_LED
void setEffectParameters(int effectIndex, NeoPixelEffects &effects, CRGB* leds, int numLeds) {
    // 前のエフェクトをクリア
    effects.stop();
    FastLED.clear();

    switch (effectIndex) {
        case 0:
            effects.setEffect(RAINBOWWAVE);   // レインボーな流れるLED
            effects.setParameters(75, CRGB::Green, FORWARD);
            break;
        case 1:
            effects.setEffect(COMET);         // 流れ星エフェクト
            effects.setParameters(75, CRGB::PaleVioletRed, FORWARD);
            effects.setRepeat(true);
            break;
        case 2:
            effects.setEffect(LARSON);        // 端まで流れて戻ってくるナイトライダー系エフェクト
            effects.setParameters(75, CRGB::Red, FORWARD);
            break;
        case 3:
            effects.setEffect(CHASE);         // LEDを1弾飛ばしで交互に光らせる工事現場系エフェクト
            effects.setParameters(120, CRGB::Blue, FORWARD);
            break;
        case 4:
            effects.setEffect(STATIC);        // 同色がチラチラ点滅するキラキラ系エフェクト
            effects.setParameters(100, CRGB::Orange, FORWARD);
            break;
        case 5:
            effects.setEffect(STROBE);        // ストロボエフェクト (定期的に全LEDピカピカ)
            effects.setParameters(100, CRGB::OrangeRed, FORWARD);
            break;
        case 6:
            effects.setEffect(SINEWAVE);      // 指定したLED弾数だけ消灯しながら流れるエフェクト (COMETの逆イメージ)
            effects.setParameters(20, CRGB::Green, FORWARD);
            break;
        case 7:
            effects.setEffect(RANDOM);        // LED全弾がランダムに全色点灯 (パリピエフェクト)
            effects.setParameters(70, CRGB::Yellow, FORWARD);
            break;
        case 8:
            effects.setEffect(FADEINOUT);     // フェードイン、フェードアウト点灯を繰り返すエフェクト (PULSEと同一)
            effects.setParameters(20, CRGB::LightGoldenrodYellow, FORWARD);
            break;
        case 9:
            effects.setEffect(NANAIRO);       // FADEINOUTエフェクトを1回ずつ色変えするエフェクト (七色に順番点灯)
            effects.setParameters(20, CRGB::Green, FORWARD);
            break;
        case 10:
            effects.setEffect(MERAMERA);      // LED全弾が暖色でメラメラと変化するエフェクト
            effects.setParameters(70, CRGB::Red, FORWARD);
            break;
        case 11:
            effects.setEffect(NONE);          // LED消灯
            effects.setParameters(10, CRGB::Red, FORWARD);
            break;
        default:
            break;
    }
}
#endif


// --------------------         2024-08-16
// 内蔵LEDエフェクトを更新する関数
#ifdef USE_INTERNAL_LED
void updateInternalLedEffect() {
        effects01.update();               // エフェクトを適用
        FastLED.show();                   // LEDの状態を更新
        esp_task_wdt_reset();             // WDTリセットをここで実行
}
// 内蔵LEDエフェクトを切り替えるタスク
void internalLedEffectTask(void *pvParameters) {
  while (true) {
    updateInternalLedEffect();
    vTaskDelay(50 / portTICK_PERIOD_MS);  // エフェクト更新の遅延
  }
}
#endif

// --------------------         2024-08-16
// NECOMIMI LEDエフェクトを更新する関数
#ifdef USE_NECOMIMI_LED
void updateNecomimiLedEffect() {
        effects02.update();               // エフェクトを適用
        FastLED.show();                   // LEDの状態を更新
        esp_task_wdt_reset();             // WDTリセットをここで実行
}
// NECOMIMI LEDエフェクトを切り替えるタスク
void necomimiLedEffectTask(void *pvParameters) {
  while (true) {
    updateNecomimiLedEffect();
    vTaskDelay(50 / portTICK_PERIOD_MS);  // エフェクト更新の遅延
  }
}
#endif


// --------------------         2024-08-16
// メモリ監視関数
#ifdef USE_MEMCHECK
void monitorHeap() {
    //Serial.print("Free heap Memory [byte]:  ");
    //Serial.println(ESP.getFreeHeap());                                  // 現在のフリーヒープサイズ
    //Serial.print("Largest free block     : ");
    //Serial.println(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));  // 最大連続ブロックサイズ
    //Serial.print("Total free blocks      : ");
    //Serial.println(heap_caps_get_free_size(MALLOC_CAP_8BIT));           // 合計フリーヒープサイズ
    //Serial.println("================================");
}

// メモリ監視タスク
void heapMonitorTask(void *pvParameters) {
    while (true) {
        monitorHeap();
        vTaskDelay(5000 / portTICK_PERIOD_MS);  // 5秒ごとに監視
    }
}
#endif


void handleRoot() {
  server.send(200, "text/html", "<html><head><meta charset=\"UTF-8\"/></head><body><pre>hello from m5stack!\n&boxdr;&boxh;&boxh;&boxh;&boxdl;\n&boxv;&#729;_&#729;&boxv;\n&boxur;&boxh;&boxh;&boxh;&boxul;</pre></body></html>");
}

void handleNotFound(){
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
//  server.send(404, "text/plain", message);
  server.send(404, "text/html", String(HEAD) + String("<body>") + message + String("</body>"));
}

void handle_speech() {
  String message = server.arg("say");
  String speaker = server.arg("voice");
  if(speaker != "") {
    TTS_PARMS = TTS_SPEAKER + speaker;
  }
  //Serial.println(message);
  ////////////////////////////////////////
  // 音声の発声
  ////////////////////////////////////////
  avatar.setExpression(Expression::Happy);
  Voicevox_tts((char*)message.c_str(), (char*)TTS_PARMS.c_str());
  server.send(200, "text/plain", String("OK"));
}


// --------------------         2024-08-16
// HTTPSをポストするタスク (SSL証明書) ※メモリ(Free heap)が少なくなると失敗する
String https_post_json(const char* url, const char* json_string, const char* root_ca) {
  String payload = "";

  // WiFiClientSecureをstd::unique_ptrで管理
  std::unique_ptr<WiFiClientSecure> client(new WiFiClientSecure());
  
  if (client) {
    client->setCACert(root_ca);
    
    // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is 
    // HTTPClientもstd::unique_ptrで管理
    std::unique_ptr<HTTPClient> https(new HTTPClient());
    https->setTimeout(65000);

    //Serial.print("[HTTPS] begin...\n");
    if (https->begin(*client, url)) {  // HTTPS
      //Serial.print("[HTTPS] POST...\n");
      // HTTPヘッダの設定   // start connection and send HTTP header
      https->addHeader("Content-Type", "application/json");
      https->addHeader("Authorization", String("Bearer ") + OPENAI_API_KEY);
      int httpCode = https->POST((uint8_t *)json_string, strlen(json_string));

      // httpCode will be negative on error
      if (httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        //Serial.printf("[HTTPS] POST... code: %d\n", httpCode);

        // file found at server
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          payload = https->getString();
          ////Serial.println("//////////////");
          //Serial.println(payload);
          //Serial.println("//////////////");
        }
      } else {
        //Serial.printf("[HTTPS] POST... failed, error: %s\n", https->errorToString(httpCode).c_str());
      }
      https->end();
    } else {
      //Serial.printf("[HTTPS] Unable to connect\n");
    }
    // End extra scoping block
  } else {
    //Serial.println("Unable to create client");
  }
  return payload;
}


String chatGpt(String json_string) {
  String response = "";;
  
  avatar.setExpression(Expression::Doubt);
  avatar.setSpeechText("考え中…");
  
  //Serial.println("AskContents::"+json_string);
  String ret = https_post_json("https://api.openai.com/v1/chat/completions", json_string.c_str(), root_ca_openai);
  avatar.setExpression(Expression::Neutral);
  avatar.setSpeechText("");
  if(ret != ""){
    DynamicJsonDocument doc(2000);
    DeserializationError error = deserializeJson(doc, ret.c_str());
    if (error) {
      //Serial.print(F("deserializeJson() failed: "));
      //Serial.println(error.f_str());
      avatar.setExpression(Expression::Sad);
      avatar.setSpeechText("エラーです");
      response = "エラーです";
      delay(1000);
      avatar.setSpeechText("");
      avatar.setExpression(Expression::Neutral);
    }else{
      const char* data = doc["choices"][0]["message"]["content"];
      //Serial.println(data);
      response = String(data);

      if(Now_mode == true){
        //ここでNow_modeがtrueの時responseをmake_fileに送る
        //Serial.println("[filemake]");
        std::replace(response.begin(),response.end(),'\n',' ');
        response = "コードを生成します";
      }
      else{
        Serial.println(response);
      }
      
    }
  } else {
    avatar.setExpression(Expression::Sad);
    avatar.setSpeechText("わかりません");
    response = "わかりません";
    delay(1000);
    avatar.setSpeechText("");
    avatar.setExpression(Expression::Neutral);
  }
  //make_file(response);

  return response;
}

//ここでファイルを作成してみる
/*
void make_file(string res){
  ofstream MyFile("generated_script.txt");
  MyFile << res;
  MyFile.close();
}
*/


String InitBuffer = "";

void handle_chat() {
  static String response = "";
  // tts_parms_no = 1;
  String text = server.arg("text");
  String speaker = server.arg("voice");
  if(speaker != "") {
    TTS_PARMS = TTS_SPEAKER + speaker;
  }
  //Serial.println(InitBuffer);
  init_chat_doc(InitBuffer.c_str());
  // 質問をチャット履歴に追加
  chatHistory.push_back(text);
   // チャット履歴が最大数を超えた場合、古い質問と回答を削除
  if (chatHistory.size() > MAX_HISTORY * 2)
  {
    chatHistory.pop_front();
    chatHistory.pop_front();
  }

  for (int i = 0; i < chatHistory.size(); i++)
  {
    JsonArray messages = chat_doc["messages"];
    JsonObject systemMessage1 = messages.createNestedObject();
    if(i % 2 == 0) {
      systemMessage1["role"] = "user";
    } else {
      systemMessage1["role"] = "assistant";
    }
    systemMessage1["content"] = chatHistory[i];
  }

  String json_string;
  serializeJson(chat_doc, json_string);
  if(speech_text=="" && speech_text_buffer == "") {
    response = chatGpt(json_string);
    speech_text = response;
    //mode = 0;
    // 返答をチャット履歴に追加
    chatHistory.push_back(response);
  } else {
    response = "busy";
  }
  // Serial.printf("chatHistory.max_size %d \n",chatHistory.max_size());
  // Serial.printf("chatHistory.size %d \n",chatHistory.size());
  // for (int i = 0; i < chatHistory.size(); i++)
  // {
  //   Serial.print(i);
  //   Serial.println("= "+chatHistory[i]);
  // }
  serializeJsonPretty(chat_doc, json_string);
  //Serial.println("====================");
  //Serial.println(json_string);
  //Serial.println("====================");
  server.send(200, "text/html", String(HEAD)+String("<body>")+response+String("</body>"));
}

void exec_chatGPT(String text) {
  static String response = "";
  //Serial.println(InitBuffer);
  init_chat_doc(InitBuffer.c_str());
  // 質問をチャット履歴に追加
  chatHistory.push_back(text);
   // チャット履歴が最大数を超えた場合、古い質問と回答を削除
  if (chatHistory.size() > MAX_HISTORY * 2)
  {
    chatHistory.pop_front();
    chatHistory.pop_front();
  }

  for (int i = 0; i < chatHistory.size(); i++)
  {
    JsonArray messages = chat_doc["messages"];
    JsonObject systemMessage1 = messages.createNestedObject();
    if(i % 2 == 0) {
      systemMessage1["role"] = "user";
    } else {
      systemMessage1["role"] = "assistant";
    }
    systemMessage1["content"] = chatHistory[i];
  }

  String json_string;
  serializeJson(chat_doc, json_string);
  if(speech_text=="" && speech_text_buffer == "") {
    response = chatGpt(json_string);
    speech_text = response;
    // 返答をチャット履歴に追加
    chatHistory.push_back(response);
  } else {
    response = "busy";
  }
  // Serial.printf("chatHistory.max_size %d \n",chatHistory.max_size());
  // Serial.printf("chatHistory.size %d \n",chatHistory.size());
  // for (int i = 0; i < chatHistory.size(); i++)
  // {
  //   Serial.print(i);
  //   Serial.println("= "+chatHistory[i]);
  // }
  serializeJsonPretty(chat_doc, json_string);
  //Serial.println("====================");
  //Serial.println(json_string);
  //Serial.println("====================");

}
/*
String Role_JSON = "";
void exec_chatGPT1(String text) {
  static String response = "";
  init_chat_doc(Role_JSON.c_str());

  String role = chat_doc["messages"][0]["role"];
  if(role == "user") {chat_doc["messages"][0]["content"] = text;}
  String json_string;
  serializeJson(chat_doc, json_string);

  response = chatGpt(json_string);
  speech_text = response;
//  server.send(200, "text/html", String(HEAD)+String("<body>")+response+String("</body>"));
}
*/
void handle_apikey() {
  // ファイルを読み込み、クライアントに送信する
  server.send(200, "text/html", APIKEY_HTML);
}

void handle_apikey_set() {
  // POST以外は拒否
  if (server.method() != HTTP_POST) {
    return;
  }
  // openai
  String openai = server.arg("openai");
  // voicetxt
  String voicevox = server.arg("voicevox");
  // voicetxt
  String sttapikey = server.arg("sttapikey");
 
  OPENAI_API_KEY = openai;
  VOICEVOX_API_KEY = voicevox;
  STT_API_KEY = sttapikey;
  //Serial.println(openai);
  //Serial.println(voicevox);
  //Serial.println(sttapikey);

  uint32_t nvs_handle;
  if (ESP_OK == nvs_open("apikey", NVS_READWRITE, &nvs_handle)) {
    nvs_set_str(nvs_handle, "openai", openai.c_str());
    nvs_set_str(nvs_handle, "voicevox", voicevox.c_str());
    nvs_set_str(nvs_handle, "sttapikey", sttapikey.c_str());
    nvs_close(nvs_handle);
  }
  server.send(200, "text/plain", String("OK"));
}

void handle_role() {
  // ファイルを読み込み、クライアントに送信する
  server.send(200, "text/html", ROLE_HTML);
}

bool save_json(){
  // SPIFFSをマウントする
  if(!SPIFFS.begin(true)){
    //Serial.println("An Error has occurred while mounting SPIFFS");
    return false;
  }

  // JSONファイルを作成または開く
  File file = SPIFFS.open("/data.json", "w");
  if(!file){
    //Serial.println("Failed to open file for writing");
    return false;
  }

  // JSONデータをシリアル化して書き込む
  serializeJson(chat_doc, file);
  file.close();
  return true;
}

/**
 * アプリからテキスト(文字列)と共にRoll情報が配列でPOSTされてくることを想定してJSONを扱いやすい形に変更
 * 出力形式をJSONに変更
*/
void handle_role_set() {
  // POST以外は拒否
  if (server.method() != HTTP_POST) {
    return;
  }
  String role = server.arg("plain");
  if (role != "") {
//    init_chat_doc(InitBuffer.c_str());
    init_chat_doc(json_ChatString.c_str());
    JsonArray messages = chat_doc["messages"];
    JsonObject systemMessage1 = messages.createNestedObject();
    systemMessage1["role"] = "system";
    systemMessage1["content"] = role;
//    serializeJson(chat_doc, InitBuffer);
  } else {
    init_chat_doc(json_ChatString.c_str());
  }
  //会話履歴をクリア
  chatHistory.clear();

  InitBuffer="";
  serializeJson(chat_doc, InitBuffer);
  //Serial.println("InitBuffer = " + InitBuffer);
  Role_JSON = InitBuffer;

  // JSONデータをspiffsへ出力する
  save_json();

  // 整形したJSONデータを出力するHTMLデータを作成する
  String html = "<html><body><pre>";
  serializeJsonPretty(chat_doc, html);
  html += "</pre></body></html>";

  // HTMLデータをシリアルに出力する
  //Serial.println(html);
  server.send(200, "text/html", html);
//  server.send(200, "text/plain", String("OK"));
};

// 整形したJSONデータを出力するHTMLデータを作成する
void handle_role_get() {

  String html = "<html><body><pre>";
  serializeJsonPretty(chat_doc, html);
  html += "</pre></body></html>";

  // HTMLデータをシリアルに出力する
  //Serial.println(html);
  server.send(200, "text/html", String(HEAD) + html);
};

void handle_face() {
  String expression = server.arg("expression");
  expression = expression + "\n";
  //Serial.println(expression);
  switch (expression.toInt())
  {
    case 0: avatar.setExpression(Expression::Neutral); break;
    case 1: avatar.setExpression(Expression::Happy); break;
    case 2: avatar.setExpression(Expression::Sleepy); break;
    case 3: avatar.setExpression(Expression::Doubt); break;
    case 4: avatar.setExpression(Expression::Sad); break;
    case 5: avatar.setExpression(Expression::Angry); break;  
  } 
  server.send(200, "text/plain", String("OK"));
}

void handle_setting() {
  String value = server.arg("volume");
  String led = server.arg("led");
  String speaker = server.arg("speaker");
//  volume = volume + "\n";
  //Serial.println(speaker);
  //Serial.println(value);
  size_t speaker_no;
  if(speaker != ""){
    speaker_no = speaker.toInt();
    if(speaker_no > 60) {
      speaker_no = 60;
    }
    TTS_SPEAKER_NO = String(speaker_no);
    TTS_PARMS = TTS_SPEAKER + TTS_SPEAKER_NO;
  }

  if(value == "") value = "180";
  size_t volume = value.toInt();
  uint8_t led_onoff = 0;
  uint32_t nvs_handle;
  if (ESP_OK == nvs_open("setting", NVS_READWRITE, &nvs_handle)) {
    if(volume > 255) volume = 255;
    nvs_set_u32(nvs_handle, "volume", volume);
    if(led != "") {
      if(led == "on") led_onoff = 1;
      else  led_onoff = 0;
      nvs_set_u8(nvs_handle, "led", led_onoff);
    }
    if(speaker != "") nvs_set_u8(nvs_handle, "speaker", speaker_no);

    nvs_close(nvs_handle);
  }
  M5.Speaker.setVolume(volume);
  M5.Speaker.setChannelVolume(m5spk_virtual_channel, volume);
  server.send(200, "text/plain", String("OK"));
}

AudioOutputM5Speaker out(&M5.Speaker, m5spk_virtual_channel);
AudioGeneratorMP3 *mp3;
AudioFileSourceBuffer *buff = nullptr;
int preallocateBufferSize = 30*1024;
uint8_t *preallocateBuffer;
AudioFileSourceHTTPSStream *file = nullptr;

void playMP3(AudioFileSourceBuffer *buff){
  mp3->begin(buff, &out);
}

// Called when a metadata event occurs (i.e. an ID3 tag, an ICY block, etc.
void MDCallback(void *cbData, const char *type, bool isUnicode, const char *string)
{
  const char *ptr = reinterpret_cast<const char *>(cbData);
  (void) isUnicode; // Punt this ball for now
  // Note that the type and string may be in PROGMEM, so copy them to RAM for printf
  char s1[32], s2[64];
  strncpy_P(s1, type, sizeof(s1));
  s1[sizeof(s1)-1]=0;
  strncpy_P(s2, string, sizeof(s2));
  s2[sizeof(s2)-1]=0;
  //Serial.printf("METADATA(%s) '%s' = '%s'\n", ptr, s1, s2);
  Serial.flush();
}

// Called when there's a warning or error (like a buffer underflow or decode hiccup)
void StatusCallback(void *cbData, int code, const char *string)
{
  const char *ptr = reinterpret_cast<const char *>(cbData);
  // Note that the string may be in PROGMEM, so copy it to RAM for printf
  char s1[64];
  strncpy_P(s1, string, sizeof(s1));
  s1[sizeof(s1)-1]=0;
  //Serial.printf("STATUS(%s) '%d' = '%s'\n", ptr, code, s1);
  Serial.flush();
}

#ifdef USE_SERVO
#define START_DEGREE_VALUE_X 90
//#define START_DEGREE_VALUE_Y 90
#define START_DEGREE_VALUE_Y 85 //
ServoEasing servo_x;
ServoEasing servo_y;
#endif

void lipSync(void *args)
{
  float gazeX, gazeY;
  int level = 0;
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
  for (;;)
  {
    level = abs(*out.getBuffer());
    if(level<100) level = 0;
    if(level > 15000)
    {
      level = 15000;
    }
    float open = (float)level/15000.0;
    avatar->setMouthOpenRatio(open);
    avatar->getGaze(&gazeY, &gazeX);
    avatar->setRotation(gazeX * 5);
    delay(50);
  }
}

bool servo_home = false;

void servo(void *args)
{
  float gazeX, gazeY;
  DriveContext *ctx = (DriveContext *)args;
  Avatar *avatar = ctx->getAvatar();
  for (;;)
  {
#ifdef USE_SERVO
    if(!servo_home)
    {
    avatar->getGaze(&gazeY, &gazeX);
    servo_x.setEaseTo(START_DEGREE_VALUE_X + (int)(15.0 * gazeX));
    if(gazeY < 0) {
      int tmp = (int)(10.0 * gazeY);
      if(tmp > 10) tmp = 10;
      servo_y.setEaseTo(START_DEGREE_VALUE_Y + tmp);
    } else {
      servo_y.setEaseTo(START_DEGREE_VALUE_Y + (int)(10.0 * gazeY));
    }
    } else {
//     avatar->setRotation(gazeX * 5);
//     float b = avatar->getBreath();
       servo_x.setEaseTo(START_DEGREE_VALUE_X); 
//     servo_y.setEaseTo(START_DEGREE_VALUE_Y + b * 5);
       servo_y.setEaseTo(START_DEGREE_VALUE_Y);
    }
    synchronizeAllServosStartAndWaitForAllServosToStop();
#endif
    delay(50);
  }
}

void Servo_setup() {
#ifdef USE_SERVO
  if (servo_x.attach(SERVO_PIN_X, START_DEGREE_VALUE_X, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
    //Serial.print("Error attaching servo x");
  }
  if (servo_y.attach(SERVO_PIN_Y, START_DEGREE_VALUE_Y, DEFAULT_MICROSECONDS_FOR_0_DEGREE, DEFAULT_MICROSECONDS_FOR_180_DEGREE) == INVALID_SERVO) {
    //Serial.print("Error attaching servo y");
  }
  servo_x.setEasingType(EASE_QUADRATIC_IN_OUT);
  servo_y.setEasingType(EASE_QUADRATIC_IN_OUT);
  setSpeedForAllServos(30);

  servo_x.setEaseTo(START_DEGREE_VALUE_X); 
  servo_y.setEaseTo(START_DEGREE_VALUE_Y);
  synchronizeAllServosStartAndWaitForAllServosToStop();
#endif
}

struct box_t
{
  int x;
  int y;
  int w;
  int h;
  int touch_id = -1;

  void setupBox(int x, int y, int w, int h) {
    this->x = x;
    this->y = y;
    this->w = w;
    this->h = h;
  }
  bool contain(int x, int y)
  {
    return this->x <= x && x < (this->x + this->w)
        && this->y <= y && y < (this->y + this->h);
  }
};
static box_t box_servo;
static box_t box_stt;
static box_t box_BtnA;
static box_t box_BtnC;


// --------------------     2024-08-16
// AIｽﾀｯｸﾁｬﾝ2の顔タッチ(左側)で内蔵LEDエフェクトを可変させるタッチイベント関連タスク
#ifdef USE_INTERNAL_LED

static box_t box_led_L;
// タッチイベントを処理するタスク
void touchEventTask_L(void *pvParameters) {
  while (true) {
    M5.update();  // M5の状態を更新

    if (M5.Touch.getCount() > 0) { // タッチが検知された場合
      auto t = M5.Touch.getDetail(); // タッチ情報を取得
      //Serial.printf("Touch detected at L (%d, %d)\n", t.x, t.y);

      if (box_led_L.contain(t.x, t.y)) { // 内蔵LEDのボックスがタッチされた場合

          // エフェクトインデックスを更新
          current_INT_EffectIndex = (current_INT_EffectIndex + 1) % 12; // 0から11までのインデックスをループ
          setEffectParameters(current_INT_EffectIndex, effects01, internal_leds, INTERNAL_LEDS);
          M5.Speaker.tone(700, 100); // タッチ音の再生          
          
      	  //Serial.printf("Internal LED effect changed to index: %d\n", current_INT_EffectIndex);

          vTaskDelay(300 / portTICK_PERIOD_MS); // タッチの連続入力を防ぐ
          continue; // ループの先頭に戻る
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // タッチイベントのポーリング頻度を調整
  }
}
#endif

// --------------------     2024-08-16
// AIｽﾀｯｸﾁｬﾝ2の顔タッチ(右側)でNECOMIMI LEDエフェクトを可変させるタッチイベント関連タスク
#ifdef USE_NECOMIMI_LED

static box_t box_led_R;
// タッチイベントを処理するタスク
void touchEventTask_R(void *pvParameters) {
  while (true) {
    M5.update();  // M5の状態を更新

    if (M5.Touch.getCount() > 0) { // タッチが検知された場合
      auto t = M5.Touch.getDetail(); // タッチ情報を取得
      //Serial.printf("Touch detected at R (%d, %d)\n", t.x, t.y);

      if (box_led_R.contain(t.x, t.y)) { // NECOMIMI LEDのボックスがタッチされた場合

          // エフェクトインデックスを更新
          current_NECO_EffectIndex = (current_NECO_EffectIndex + 1) % 12; // 0から11までのインデックスをループ
          setEffectParameters(current_NECO_EffectIndex, effects02, pb_leds, PORT_B_LEDS);
          M5.Speaker.tone(700, 100); // タッチ音の再生
		  
      	  //Serial.printf("Necomimi LED effect changed to index: %d\n", current_NECO_EffectIndex);

          vTaskDelay(300 / portTICK_PERIOD_MS); // タッチの連続入力を防ぐ
          continue; // ループの先頭に戻る
      }
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // タッチイベントのポーリング頻度を調整
  }
}
#endif


void Wifi_setup() {
  // 前回接続時情報で接続する
  while (WiFi.status() != WL_CONNECTED) {
    M5.Display.print(".");
    //Serial.print(".");
    delay(500);
    // 10秒以上接続できなかったら抜ける
    if ( 10000 < millis() ) {
      break;
    }
  }
  M5.Display.println("");
  //Serial.println("");
  // 未接続の場合にはSmartConfig待受
  if ( WiFi.status() != WL_CONNECTED ) {
    WiFi.mode(WIFI_STA);
    WiFi.beginSmartConfig();
    M5.Display.println("Waiting for SmartConfig");
    //Serial.println("Waiting for SmartConfig");
    while (!WiFi.smartConfigDone()) {
      delay(500);
      M5.Display.print("#");
      //Serial.print("#");
      // 30秒以上接続できなかったら抜ける
      if ( 30000 < millis() ) {
        //Serial.println("");
        //Serial.println("Reset");
        ESP.restart();
      }
    }
    // Wi-fi接続
    M5.Display.println("");
    //Serial.println("");
    M5.Display.println("Waiting for WiFi");
    //Serial.println("Waiting for WiFi");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      M5.Display.print(".");
      //Serial.print(".");
      // 60秒以上接続できなかったら抜ける
      if ( 60000 < millis() ) {
        //Serial.println("");
        //Serial.println("Reset");
        ESP.restart();
      }
    }
  }
}

String SpeechToText(bool isGoogle){
  //Serial.println("\r\nRecord start!\r\n");

  String ret = "";
  if( isGoogle) {
    Audio* audio = new Audio();
    audio->Record();  
    //Serial.println("Record end\r\n");
    //Serial.println("音声認識開始");
    avatar.setSpeechText("わかりました");  
    CloudSpeechClient* cloudSpeechClient = new CloudSpeechClient(root_ca_google, STT_API_KEY.c_str());
    ret = cloudSpeechClient->Transcribe(audio);
    delete cloudSpeechClient;
    delete audio;
  } else {
    AudioWhisper* audio = new AudioWhisper();
    audio->Record();  
    //Serial.println("Record end\r\n");
    //Serial.println("音声認識開始");
    avatar.setSpeechText("わかりました");  
    Whisper* cloudSpeechClient = new Whisper(root_ca_openai, OPENAI_API_KEY.c_str());
    ret = cloudSpeechClient->Transcribe(audio);
    delete cloudSpeechClient;
    delete audio;
  }
  return ret;
}

void setup()
{
  Serial.begin(115200);       // デバッグ用のシリアル通信開始
  auto cfg = M5.config();

  cfg.external_spk = true;    /// use external speaker (SPK HAT / ATOMIC SPK)
  // cfg.external_spk_detail.omit_atomic_spk = true; // exclude ATOMIC SPK
  // cfg.external_spk_detail.omit_spk_hat    = true; // exclude SPK HAT
  // cfg.output_power = true;
  M5.begin(cfg);

  preallocateBuffer = (uint8_t *)malloc(preallocateBufferSize);
  if (!preallocateBuffer) {
    M5.Display.printf("FATAL ERROR:  Unable to preallocate %d bytes for app\n", preallocateBufferSize);
    for (;;) { delay(1000); }
  }
{
  auto micConfig = M5.Mic.config();
  micConfig.stereo = false;
  micConfig.sample_rate = 16000;
  M5.Mic.config(micConfig);
}
  M5.Mic.begin();

  { /// custom setting
    auto spk_cfg = M5.Speaker.config();
    /// Increasing the sample_rate will improve the sound quality instead of increasing the CPU load.
    // 音声は96 → 64KHzに変更 2024-08-16
    spk_cfg.sample_rate = 64000; // default:64000 (64kHz)  e.g. 48000 , 50000 , 80000 , 96000 , 100000 , 128000 , 144000 , 192000 , 200000
    spk_cfg.task_pinned_core = APP_CPU_NUM;
    M5.Speaker.config(spk_cfg);
  }
//  M5.Speaker.begin();


// --------------------			2024-08-16
// LEDエフェクトをマルチタスクで実行する
#ifdef USE_LED

#ifdef USE_INTERNAL_LED
  FastLED.setBrightness(30);
  FastLED.addLeds<NEOPIXEL, INTERNAL_PIN>(internal_leds, INTERNAL_LEDS);

  // 起動時に初期エフェクトを設定
  setEffectParameters(current_INT_EffectIndex, effects01, internal_leds, INTERNAL_LEDS);
  FastLED.show(); // 初期状態のLEDを反映

  // 内蔵LEDエフェクトタスクをコア1に作成
  xTaskCreatePinnedToCore(
    internalLedEffectTask,      // タスク関数
    "Internal LED Effect Task", // タスク名
    1536,                       // スタックサイズ
    NULL,                       // 引数
    2,                          // 優先度
    NULL,                       // タスクハンドル
    1                           // コアID（0または1）
  );
  
  // タッチイベントタスク(右側)をコア0に作成
  xTaskCreatePinnedToCore(
    touchEventTask_L,           // タスク関数
    "Touch Event Task L",       // タスク名
    3072,                       // スタックサイズ
    NULL,                       // 引数
    2,                          // 優先度
    NULL,                       // タスクハンドル
    1                           // コアID（0または1）
  ); 
#endif

#ifdef USE_NECOMIMI_LED
  FastLED.setBrightness(30);
  FastLED.addLeds<NEOPIXEL, PORT_B_PIN>(pb_leds, PORT_B_LEDS);

  // 起動時に初期エフェクトを設定
  setEffectParameters(current_NECO_EffectIndex, effects02, pb_leds, PORT_B_LEDS);
  FastLED.show(); // 初期状態のLEDを反映

  // NECOMIMI LEDエフェクトタスクをコア1に作成
  xTaskCreatePinnedToCore(
    necomimiLedEffectTask,      // タスク関数
    "Necomimi LED Effect Task", // タスク名
    1536,                       // スタックサイズ
    NULL,                       // 引数
    2,                          // 優先度
    NULL,                       // タスクハンドル
    1                           // コアID（0または1）
  );

  // タッチイベントタスクをコア0に作成
  xTaskCreatePinnedToCore(
    touchEventTask_R,           // タスク関数
    "Touch Event Task R",       // タスク名
    3072,                       // スタックサイズ
    NULL,                       // 引数
    2,                          // 優先度
    NULL,                       // タスクハンドル
    1                           // コアID（0または1）
  ); 
#endif

#endif


  Servo_setup();
  delay(1000);

  {
    uint32_t nvs_handle;
    if (ESP_OK == nvs_open("setting", NVS_READONLY, &nvs_handle)) {
      size_t volume;
      uint8_t led_onoff;
      uint8_t speaker_no;
      nvs_get_u32(nvs_handle, "volume", &volume);
      if(volume > 255) volume = 255;
      M5.Speaker.setVolume(volume);
      M5.Speaker.setChannelVolume(m5spk_virtual_channel, volume);
      nvs_get_u8(nvs_handle, "led", &led_onoff);
      // if(led_onoff == 1) Use_LED = true;
      // else  Use_LED = false;
      nvs_get_u8(nvs_handle, "speaker", &speaker_no);
      if(speaker_no > 60) speaker_no = 3;
      TTS_SPEAKER_NO = String(speaker_no);
      TTS_PARMS = TTS_SPEAKER + TTS_SPEAKER_NO;
      nvs_close(nvs_handle);
    } else {
      if (ESP_OK == nvs_open("setting", NVS_READWRITE, &nvs_handle)) {
        size_t volume = 180;
        uint8_t led_onoff = 0;
        uint8_t speaker_no = 3;
        nvs_set_u32(nvs_handle, "volume", volume);
        nvs_set_u8(nvs_handle, "led", led_onoff);
        nvs_set_u8(nvs_handle, "speaker", speaker_no);
        nvs_close(nvs_handle);
        M5.Speaker.setVolume(volume);
        M5.Speaker.setChannelVolume(m5spk_virtual_channel, volume);
        // Use_LED = false;
        TTS_SPEAKER_NO = String(speaker_no);
        TTS_PARMS = TTS_SPEAKER + TTS_SPEAKER_NO;
      }
    }
  }

  M5.Lcd.setTextSize(2);
  //Serial.println("Connecting to WiFi");
  WiFi.disconnect();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
#ifndef USE_SDCARD
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  OPENAI_API_KEY = String(OPENAI_APIKEY);
  VOICEVOX_API_KEY = String(VOICEVOX_APIKEY);
  STT_API_KEY = String(STT_APIKEY);
#else
  /// settings
  if (SD.begin(GPIO_NUM_4, SPI, 25000000)) {
    //prompt
    File pyfile;
    pyfile = SD.open("/prompt.txt", FILE_READ);
    if(pyfile)
    {
      size_t sz = pyfile.size();
      char buf[sz + 1];
      pyfile.read((uint8_t*)buf, sz);
      buf[sz] = 0;
      pyfile.close();
      pre_prompt=buf;
    }

    //Serial.println("PROMPT"+pre_prompt);
    pyfile.close();

    /// wifi
    auto fs = SD.open("/wifi.txt", FILE_READ);
    if(fs) {
      size_t sz = fs.size();
      char buf[sz + 1];
      fs.read((uint8_t*)buf, sz);
      buf[sz] = 0;
      fs.close();

      int y = 0;
      for(int x = 0; x < sz; x++) {
        if(buf[x] == 0x0a || buf[x] == 0x0d)
          buf[x] = 0;
        else if (!y && x > 0 && !buf[x - 1] && buf[x])
          y = x;
      }
      WiFi.begin(buf, &buf[y]);
    } else {
       WiFi.begin();
    }

    uint32_t nvs_handle;
    if (ESP_OK == nvs_open("apikey", NVS_READWRITE, &nvs_handle)) {
      /// radiko-premium
      fs = SD.open("/apikey.txt", FILE_READ);
      if(fs) {
        size_t sz = fs.size();
        char buf[sz + 1];
        fs.read((uint8_t*)buf, sz);
        buf[sz] = 0;
        fs.close();
  
        int y = 0;
        int z = 0;
        for(int x = 0; x < sz; x++) {
          if(buf[x] == 0x0a || buf[x] == 0x0d)
            buf[x] = 0;
          else if (!y && x > 0 && !buf[x - 1] && buf[x])
            y = x;
          else if (!z && x > 0 && !buf[x - 1] && buf[x])
            z = x;
        }

        nvs_set_str(nvs_handle, "openai", buf);
        nvs_set_str(nvs_handle, "voicevox", &buf[y]);
        nvs_set_str(nvs_handle, "sttapikey", &buf[z]);
        //Serial.println("------------------------");
        //Serial.println(buf);
        //Serial.println(&buf[y]);
        //Serial.println(&buf[z]);
        //Serial.println("------------------------");
      }
      
      nvs_close(nvs_handle);
    }
    SD.end();
  } else {
    WiFi.begin();
  }

  {
    uint32_t nvs_handle;
    if (ESP_OK == nvs_open("apikey", NVS_READONLY, &nvs_handle)) {
      //Serial.println("nvs_open");

      size_t length1;
      size_t length2;
      size_t length3;
      if(ESP_OK == nvs_get_str(nvs_handle, "openai", nullptr, &length1) && 
         ESP_OK == nvs_get_str(nvs_handle, "voicevox", nullptr, &length2) && 
         ESP_OK == nvs_get_str(nvs_handle, "sttapikey", nullptr, &length3) && 
        length1 && length2 && length3) {
        //Serial.println("nvs_get_str");
        char openai_apikey[length1 + 1];
        char voicevox_apikey[length2 + 1];
        char stt_apikey[length3 + 1];
        if(ESP_OK == nvs_get_str(nvs_handle, "openai", openai_apikey, &length1) && 
           ESP_OK == nvs_get_str(nvs_handle, "voicevox", voicevox_apikey, &length2) &&
           ESP_OK == nvs_get_str(nvs_handle, "sttapikey", stt_apikey, &length3)) {
          OPENAI_API_KEY = String(openai_apikey);
          VOICEVOX_API_KEY = String(voicevox_apikey);
          STT_API_KEY = String(stt_apikey);
          // Serial.println(OPENAI_API_KEY);
          // Serial.println(VOICEVOX_API_KEY);
          // Serial.println(STT_API_KEY);
        }
      }
      nvs_close(nvs_handle);
    }
  }
  
#endif

  M5.Lcd.print("Connecting");
  Wifi_setup();
  M5.Lcd.println("\nConnected");

#ifdef ENABLE_MDNS
  if (MDNS.begin(MDNS_HOST)) {
    //Serial.println("MDNS responder started");
    M5.Lcd.println("MDNS responder started");
  }
#endif

  //Serial.printf_P(PSTR("Go to http://"));
  M5.Lcd.print("Go to http://");
  //Serial.println(WiFi.localIP());
  M5.Lcd.println(WiFi.localIP());

#ifdef ENABLE_MDNS
  //Serial.printf_P(PSTR("or http://%s.local"), PSTR(MDNS_HOST));
  //Serial.println();
  M5.Lcd.printf("or http://%s.local", MDNS_HOST);
  M5.Lcd.println();
#endif

  delay(1000);

  server.on("/", handleRoot);

  server.on("/inline", [](){
    server.send(200, "text/plain", "this works as well");
  });

  // And as regular external functions:
  server.on("/speech", handle_speech);
  server.on("/face", handle_face);
  server.on("/chat", handle_chat);
  server.on("/apikey", handle_apikey);
  server.on("/setting", handle_setting);
  server.on("/apikey_set", HTTP_POST, handle_apikey_set);
  server.on("/role", handle_role);
  server.on("/role_set", HTTP_POST, handle_role_set);
  server.on("/role_get", handle_role_get);
  server.onNotFound(handleNotFound);

  init_chat_doc(json_ChatString.c_str());
  // SPIFFSをマウントする
  if(SPIFFS.begin(true)){
    // JSONファイルを開く
    File file = SPIFFS.open("/data.json", "r");
    if(file){
      DeserializationError error = deserializeJson(chat_doc, file);
      if(error){
        //Serial.println("Failed to deserialize JSON");
        init_chat_doc(json_ChatString.c_str());
      }
      serializeJson(chat_doc, InitBuffer);
      Role_JSON = InitBuffer;
      String json_str; 
      serializeJsonPretty(chat_doc, json_str);  // 文字列をシリアルポートに出力する
      //Serial.println(json_str);
    } else {
      //Serial.println("Failed to open file for reading");
      init_chat_doc(json_ChatString.c_str());
    }
  } else {
    //Serial.println("An Error has occurred while mounting SPIFFS");
  }

  server.begin();
  //Serial.println("HTTP server started");
  M5.Lcd.println("HTTP server started");  
  
  //Serial.printf_P(PSTR("/ to control the chatGpt Server.\n"));
  M5.Lcd.print("/ to control the chatGpt Server.\n");
  delay(3000);

  audioLogger = &Serial;
  mp3 = new AudioGeneratorMP3();
//  mp3->RegisterStatusCB(StatusCallback, (void*)"mp3");
  wakeword_init();

  avatar.init();
  avatar.addTask(lipSync, "lipSync");
  avatar.addTask(servo, "servo");
  avatar.setSpeechFont(&fonts::efontJA_16);

//  M5.Speaker.setVolume(200);


// --------------------         2024-08-16
// AIｽﾀｯｸﾁｬﾝ2の顔タッチの座標情報

  box_stt.setupBox(0, 0, M5.Display.width(), 60);   
  box_servo.setupBox(107, 80, 106, 60);
  box_BtnA.setupBox(0, 80, 60, 60);                 // 座標修正 2024-08-16 ※初期値より大きさ修正
  box_BtnC.setupBox(260, 80, 60, 60);               // 座標修正 2024-08-16 ※初期値より大きさ修正

#ifdef USE_INTERNAL_LED
  box_led_L.setupBox(0, 160, 150, 80);              // LEDタッチ座標定義 (左下) 2024-08-16 ※中央スペース有 (20px)
#endif
#ifdef USE_NECOMIMI_LED
  box_led_R.setupBox(170, 160, 150, 80);            // LEDタッチ座標定義 (右下) 2024-08-16 ※中央スペース有 (20px)
#endif

#ifdef USE_MEMCHECK
// --------------------         2024-08-16
// メモリ監視タスクを起動
  xTaskCreate(
	  heapMonitorTask, 		  // タスク関数
	  "Heap Monitor Task", 	// タスク名
	  2048, 					      // スタックサイズ
	  NULL, 					      // 引数
	  1, 						        // 優先度
	  NULL					        // タスクハンドル
  );
#endif


}


// --------------------         2024-08-16
// 『お腹空いた』だとChatGPTは【あいた】と回答してしまう為、わざとひらがなにて【すいた】に変更しました。
String random_words[18] = {"あなたは誰","楽しい","怒った","可愛い","悲しい","眠い","ジョークを言って","泣きたい","怒ったぞ","こんにちは","お疲れ様","詩を書いて","疲れた","お腹がすいた","嫌いだ","苦しい","俳句を作って","歌をうたって"};
int random_time = -1;
bool random_speak = true;
static int lastms1 = 0;

void report_batt_level(){
  mode = 0;
  char buff[100];
   int level = M5.Power.getBatteryLevel();
  if(M5.Power.isCharging())
    sprintf(buff,"充電中、バッテリーのレベルは%d％です。",level);
  else
    sprintf(buff,"バッテリーのレベルは%d％です。",level);
  avatar.setExpression(Expression::Happy);
  mode = 0; 
  speech_text = String(buff);
  delay(1000);
  avatar.setExpression(Expression::Neutral);
}

void switch_monologue_mode(){
    String tmp;
    mode = 0;
    if(random_speak) {
      tmp = "独り言始めます。";
      lastms1 = millis();
      random_time = 40000 + 1000 * random(30);
    } else {
      tmp = "独り言やめます。";
      random_time = -1;
    }
    random_speak = !random_speak;
    avatar.setExpression(Expression::Happy);
    mode = 0;
    speech_text = tmp;
    delay(1000);
    avatar.setExpression(Expression::Neutral);
}

void sw_tone(){
      M5.Mic.end();
        M5.Speaker.tone(1000, 100);
      delay(500);
        M5.Speaker.end();
      M5.Mic.begin();
}

void SST_ChatGPT() {
        bool prev_servo_home = servo_home;
        random_speak = true;
        random_time = -1;
#ifdef USE_SERVO
        servo_home = true;
#endif
        avatar.setExpression(Expression::Happy);
        avatar.setSpeechText("御用でしょうか？");
        String ret;
        if(OPENAI_API_KEY != STT_API_KEY){
          //Serial.println("Google STT");
          ret = SpeechToText(true);
        } else {
          //Serial.println("Whisper STT");
          ret = SpeechToText(false);
        }
#ifdef USE_SERVO
        servo_home = prev_servo_home;
#endif
        //Serial.println("音声認識終了");
        //Serial.println("音声認識結果");
        if(ret != "") {
          Serial.println(ret);
          if (!mp3->isRunning() && speech_text=="" && speech_text_buffer == "") {
            if(Now_mode==true)
            {
              //Serial.println("INPUTWORD::"+ret);
              //ret=pre_prompt+ret;
              ret=pre_prompt+ret;
              Serial.println("!!"+ret);
            }
            exec_chatGPT(ret);
            mode = 0;
          }
        } else {
          //Serial.println("音声認識失敗");
          avatar.setExpression(Expression::Sad);
          avatar.setSpeechText("聞き取れませんでした");
          delay(2000);
          avatar.setSpeechText("");
          avatar.setExpression(Expression::Neutral);
        } 
}


void loop()
{
  static int lastms = 0;

  if (random_time >= 0 && millis() - lastms1 > random_time)
  {
    lastms1 = millis();
    random_time = 40000 + 1000 * random(30);
    if (!mp3->isRunning() && speech_text=="" && speech_text_buffer == "") {
      exec_chatGPT(random_words[random(18)]);
      mode = 0;
    }
  }

  M5.update();
  if (M5.BtnA.wasPressed())
  {
    if(mode >= 0){
      sw_tone();
      if(mode == 0){
        avatar.setSpeechText("ウェイクワード有効");
        mode = 1;
        wakeword_is_enable = true;
      } else {
        avatar.setSpeechText("ウェイクワード無効");
        mode = 0;
        wakeword_is_enable = false;
      }
      delay(1000);
      avatar.setSpeechText("");
    }
  }

  if (M5.BtnB.pressedFor(2000)) {
     M5.Mic.end();
     M5.Speaker.tone(1000, 100);
     delay(500);
     M5.Speaker.tone(600, 100);
     delay(1000);
    M5.Speaker.end();
    M5.Mic.begin();
    random_time = -1;           //独り言停止
    random_speak = true;
    wakeword_is_enable = false; //wakeword 無効
    mode = -1;
#ifdef USE_SERVO
      servo_home = true;
      delay(500);
#endif
    avatar.setSpeechText("ウェイクワード登録開始");
  }

//ボタンCを押すとコード生成モードに移行する機能を追加
  if (M5.BtnC.wasPressed())
  {
    //Serial.println("[mode changed]");
    Now_mode = !Now_mode;
    //Serial.println(Now_mode);
    if(Now_mode == true){
      avatar.setSpeechText("コード生成モード");
    }else{
      avatar.setSpeechText("おしゃべりモード");
    }
    sw_tone();
    //report_batt_level();
  }

#if defined(ARDUINO_M5STACK_Core2) || defined( ARDUINO_M5STACK_CORES3 )
  auto count = M5.Touch.getCount();
  if (count)
  {
    auto t = M5.Touch.getDetail();
    if (t.wasPressed())
    {          
      if (box_stt.contain(t.x, t.y)&&(!mp3->isRunning()))
      {
        sw_tone();
        SST_ChatGPT();
      }
#ifdef USE_SERVO
      if (box_servo.contain(t.x, t.y))
      {
        servo_home = !servo_home;
        M5.Speaker.tone(1000, 100);
      }
#endif
      if (box_BtnA.contain(t.x, t.y))
      {
        sw_tone();
        switch_monologue_mode();
      }
      if (box_BtnC.contain(t.x, t.y))
      {
        sw_tone();
        report_batt_level();
      }
    }
  }
#endif

  if(speech_text != ""){
    avatar.setExpression(Expression::Happy);
    speech_text_buffer = speech_text;
    speech_text = "";
    M5.Mic.end();
    M5.Speaker.begin();
    mode = 0;
    Voicevox_tts((char*)speech_text_buffer.c_str(), (char*)TTS_PARMS.c_str());
  }

  if (mp3->isRunning()) {
    if (!mp3->loop()) {
      mp3->stop();
      if(file != nullptr){delete file; file = nullptr;}
      //Serial.println("mp3 stop");
      avatar.setExpression(Expression::Neutral);
      speech_text_buffer = "";
      delay(200);
      M5.Speaker.end();
      M5.Mic.begin();
      if(wakeword_is_enable) mode = 1;
      else  mode = 0;
    }
    delay(1);
  } else {
  server.handleClient();
  }

  if (mode == 0) { return; }
  else if (mode < 0) {
    if(wakeword_regist()){
      avatar.setSpeechText("ウェイクワード登録終了");
      delay(1000);
      avatar.setSpeechText("");
      mode = 0;
      wakeword_is_enable = false;
#ifdef USE_SERVO
      servo_home = false;
#endif
    }
  }
  else if (mode > 0 && wakeword_is_enable) {
    if( wakeword_compare()){
        //Serial.println("wakeword_compare OK!");
        sw_tone();
        SST_ChatGPT();
    }
  }
}
