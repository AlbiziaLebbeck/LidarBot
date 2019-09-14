#include <M5Stack.h>
#include <WiFi.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "lidarcar.h"
#include "rprtrack.h"
#include "iic.h"

#define WIFI_STA_NAME "LidarBot"
#define WIFI_STA_PASS "lidarbot"

IPAddress server(192,168,43,92);
const uint16_t serverPort = 11411;
ros::NodeHandle  nh;

I2C i2c;
LidarCar lidarcar;

extern const unsigned char gImage_logo[];

unsigned long pub_map_time = millis() + 100;

void cmd_velCallback(const geometry_msgs::Twist& CVel);
ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", &cmd_velCallback);

sensor_msgs::LaserScan scan_msg;
ros::Publisher scan("scan", &scan_msg);

unsigned long lastscan_time;

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

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  nh.subscribe(cmd_vel);
  nh.advertise(scan);
  scan_msg.ranges = (float*)realloc(scan_msg.ranges, 360 * sizeof(float));
  scan_msg.intensities = (float*)realloc(scan_msg.intensities, 360 * sizeof(float));
  //  

  M5.Lcd.setCursor(240, 220, 2);    
  M5.Lcd.printf("mode");

  //!Motor
  lidarcar.Init();

  //!Camrea
  i2c.master_start();

  lastscan_time = millis();
  
}

uint8_t offset = 0;

void loop() {
  // put your main code here, to run repeatedly:
  M5.Lcd.setCursor(240, 1, 4);    
  M5.Lcd.printf("ROS_lidarbot");
  
  lidarcar.MapDisplay();
  i2c.master_hangs();

  if (lidarcar.startAngle == 33750) {

    unsigned long scan_time = 0.001*(millis() - lastscan_time);
    lastscan_time = millis();
    
    scan_msg.header.stamp = nh.now();
    scan_msg.header.frame_id = "laser";
    scan_msg.angle_min = (30)*3.14159/180;
    scan_msg.angle_max = (149)*3.14159/180;
    scan_msg.angle_increment = 1*3.14159/180;
    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / 120;
    scan_msg.range_min = 0.07;
    scan_msg.range_max = 20.0;
    scan_msg.ranges_length = 120;
    scan_msg.intensities_length = 120;

    for(uint16_t i = 0; i < 120; i++){
      scan_msg.ranges[i] = lidarcar.distance[359-i-30]*0.25*0.001;
      scan_msg.intensities[i] = 10;
    }
    offset = (offset + 1)%4; 
    
    scan.publish(&scan_msg);
  }

//  scan_msg.header.stamp = nh.now();
//  scan_msg.header.frame_id = "laser_frame";
//  scan_msg.angle_min = (lidarcar.startAngle*0.01)*3.14159/180;
//  scan_msg.angle_max = (lidarcar.startAngle*0.01+22.5)*3.14159/180;
//  scan_msg.angle_increment = (22.5 / lidarcar.dataLength)*3.14159/180;
//  scan_msg.scan_time = 1/(16*lidarcar.lidarSpeed*0.05);
//  scan_msg.time_increment = 1/(16*lidarcar.lidarSpeed*0.05) / (lidarcar.dataLength);
//  scan_msg.range_min = 0.0005;
//  scan_msg.range_max = 20.0;
//  scan_msg.ranges_length = lidarcar.dataLength;
//  scan_msg.intensities_length = lidarcar.dataLength;
//
//  for(uint8_t i = 0; i < lidarcar.dataLength && i < 60; i++){
//     uint16_t ang = (uint16_t)(lidarcar.startAngle*0.01 + 22.5*i/lidarcar.dataLength);
//     scan_msg.ranges[i] = lidarcar.distance[ang]*0.25*0.001;
//     scan_msg.intensities[i] = 10;
//  }
//  scan.publish(&scan_msg);

  nh.spinOnce();
  
}

void cmd_velCallback(const geometry_msgs::Twist& CVel) {
  int8_t vel_x = (int8_t)CVel.linear.x;
  int8_t vel_th = -(int8_t)(CVel.angular.z);

  Serial.print("u: ");
  Serial.print(vel_x);
  Serial.print(" ");
  Serial.println(vel_th);

  lidarcar.ControlWheel(vel_th, vel_x, 0);
}
