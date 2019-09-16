#include <M5Stack.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "lidarcar.h"
#include "rprtrack.h"
#include "iic.h"

#define ROBOT_ID   "/ldb01"

#define WIFI_STA_NAME "LidarBot"
#define WIFI_STA_PASS "lidarbot"
//#define WIFI_STA_NAME "MrPrukWiFi"
//#define WIFI_STA_PASS "12345678"

#define MQTT_SERVER   "103.20.207.171"
#define MQTT_PORT     1883

I2C i2c;
LidarCar lidarcar;

WiFiClient client;
PubSubClient mqtt(client);

extern const unsigned char gImage_logo[];

unsigned long pub_time = millis() + 100;

void callback(char* topic, byte* payload, unsigned int length);


void setup() {
  // put your setup code here, to run once:
  m5.begin();
  Serial1.begin(230400, SERIAL_8N1, 16, 2);  //Lidar
  Serial2.begin(115200);                     //motor

  //!logo
  M5.Lcd.fillScreen(TFT_BLACK);
  m5.lcd.pushImage(0, 0, 320, 240, (uint16_t *)gImage_logo);
  M5.Lcd.setCursor(240, 1, 4);    
  M5.Lcd.printf("V 0.0.3");
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
  mqtt.setCallback(callback);
  //  

  M5.Lcd.setCursor(240, 220, 2);    
  M5.Lcd.printf("mode");

  //!Motor
  lidarcar.Init();

  //!Camrea
  i2c.master_start();
  
}

uint8_t pub_id = 0;

void loop() {
  // put your main code here, to run repeatedly:
  M5.Lcd.setCursor(240, 1, 4);    
  M5.Lcd.printf(ROBOT_ID);
  
  lidarcar.MapDisplay();
  i2c.master_hangs();

  String topic = String(ROBOT_ID) + String("/sensor");

  if (mqtt.connected() == false) {
    lidarcar.ControlWheel(0, 0, 0);
    Serial.print("MQTT connection... ");

    topic = String(ROBOT_ID) + String("/control");
    
    if (mqtt.connect("LidarBot-001")) {
      Serial.println("connected");
      
      mqtt.subscribe(topic.c_str());
    } else {
      Serial.println("failed");
      delay(5000);
    }
  } else {
    mqtt.loop();
    
    if(millis() > pub_time){

      uint8_t pub_buffer[729];
      pub_buffer[0] = pub_id;
      pub_id = (pub_id + 1) % 128;
      for (int i=0 ; i < 360 ; i++) {
        pub_buffer[i*2+2] = (lidarcar.distance[i] >> 8);
        pub_buffer[i*2+1] = (lidarcar.distance[i] & 0xFF);
      }

      for (int i=0 ; i < 4 ; i++) {
        pub_buffer[i*2+722] = (lidarcar.angleVelocity[i] >> 8);
        pub_buffer[i*2+721] = (lidarcar.angleVelocity[i] & 0xFF);
      }
      
      if(mqtt.publish(topic.c_str(), pub_buffer,729,true))
        Serial.println("SendOK");

      pub_time = millis() + 100;
    }
  } 
  
}

void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  
  String topic_str = topic, payload_str = (char*)payload;
  Serial.println("[" + topic_str + "]: " + payload_str);

  StaticJsonDocument<256> jsonBuffer;
  DeserializationError error = deserializeJson(jsonBuffer, payload, length);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
  }
  else{
    int data[3] = {jsonBuffer["wheel"][0],jsonBuffer["wheel"][1],jsonBuffer["wheel"][2]};
    
    lidarcar.ControlWheel(data[0], data[1], data[2]);
  }
}
