#include<M5Stack.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "keyboard.h"

#define ROBOT_ID   "/ldb05"

#define WIFI_STA_NAME "LidarBot"
#define WIFI_STA_PASS "lidarbot"

#define MQTT_SERVER   "103.20.207.171"
#define MQTT_PORT     1883

KeyBoard keyboard;

char command[50];

WiFiClient client;
PubSubClient mqtt(client);

extern const unsigned char gImage_logo[];

uint8_t old[3] = {0,0,0};

void setup() {
  m5.begin();
  ledcDetachPin(SPEAKER_PIN);
  digitalWrite(SPEAKER_PIN, 0);

  //!logo
  M5.Lcd.fillScreen(TFT_BLACK);
  m5.lcd.pushImage(0, 0, 320, 240, (uint16_t *)gImage_logo);
  M5.Lcd.setCursor(240, 1, 4);    
  //M5.Lcd.printf("V 0.0.2");
  delay(2000);
  M5.Lcd.fillScreen(TFT_BLACK);

  //!service
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_STA_NAME, WIFI_STA_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  //  

  //!key
  keyboard.Init();

}

uint8_t led[5] = {0x03,0x03,0x03,0x03,0x03};
void loop()
{
  M5.Lcd.setCursor(240, 1, 4);    
  M5.Lcd.printf(ROBOT_ID);
  
  if (mqtt.connected() == false) {
    Serial.print("MQTT connection... ");
    if (mqtt.connect("Remote-001")) {
      Serial.println("connected");
    } else {
      Serial.println("failed");
      delay(5000);
    }
  } else {
    mqtt.loop();
  }

  String topic = String(ROBOT_ID) + String("/control");
  keyboard.GetValue();
  if (keyboard.keyData[0] != old[0] || keyboard.keyData[1] != old[1] || keyboard.keyData[2] != old[2]) {
    sprintf(command, "{\"wheel\":[%d,%d,%d]}", keyboard.keyData[0], keyboard.keyData[1], keyboard.keyData[2]);
    mqtt.publish(topic.c_str(),command);
    old[0] = keyboard.keyData[0];
    old[1] = keyboard.keyData[1];
    old[2] = keyboard.keyData[2];
  }
}
