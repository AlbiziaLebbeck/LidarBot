uint8_t message[6] = {0xAA, 0x07, 0x07, 0x07, 0x07, 0x55};

void setup() {                

  Serial.begin(115200);
  Serial2.begin(115200);

  Serial.println("motor run!!");
  Serial2.write(message, sizeof(message));
}



void loop() {
  

  
}
