#include <Arduino.h>
#include <ODriveTeensyCAN.h>

ODriveTeensyCAN odrive;

void setup(){
  Serial.begin(115200);
  delay(1000);
  odrive.RunState(0, 8);
}

void loop(){
  Serial.println(odrive.GetVBus(0));
  vTaskDelay(10);
}