/*
python側で!!から始まり、改行で終わるまでをプロンプトとして受け付けている
*/

#include <Arduino.h>
#include <M5Unified.h>

void setup()
{
  Serial.begin(115200);
  // M5.update();
  // Serial.print("forward 0.1m/s");
}
void loop()
{
  Serial.println("!!前に進んで止まって");
  Serial.println("hello world");
  delay(3000);
}