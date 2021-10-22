
#include <arduino.h>
#include <ESP_Arduino_CAN.h>


ESP_Arduino_CAN Odrives;

void setup()
{
    Serial.begin(115200);
    Odrives.RunStateCloseLoop(0);
}

void loop()
{
    Odrives.update();
    Odrives.feedUpdate(100);

  Serial.print("  iq: ");
      Serial.print(Odrives.AXES[0].IQ.measured);
   Serial.print(" v "); 


   Serial.println(Odrives.Voltage);

}

