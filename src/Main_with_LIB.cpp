
#include <arduino.h>
#include <ESP_Arduino_CAN.h>


ESP_Arduino_CAN Odrives;
long janela;
void setup()
{
    Serial.begin(115200);
    vTaskDelay(1000);
    Odrives.RunStateCloseLoop(0);

    Odrives.SetCtrlMode(0, 2, 2);
    //Odrives.SetLimits(0, 5, 5);
    vTaskDelay(1000);

    //Odrives.SetCtrlModePos(0);
    janela =millis();
}

bool set1 = false;
bool set2 = false;
bool set3 = false;


void loop()
{
     Odrives.update();
    Odrives.feedUpdate(11);
Odrives.Debug(0);

   if (millis() > 1000 and set1 == false){
    //Odrives.SetCtrlMode(0, 2, 1);
       Serial.println("CMD_____Vel");
       Odrives.SetVelocity(0, 10);
       set1 = true;
   }


   if (millis() > 8000 and set2 == false){
       Serial.print("CMD_____TRAJ2");

       Odrives.SetTrajPos (0, -250);
       set2 = true;
   }
    
    if (millis() > 20000 and set3 == false){
       Serial.print("CMD_____TRAJ2");
        // Odrives.setTrajdec(0, 15);
        // Odrives.setTrajVel(0, 15);
       Odrives.SetTrajPos (0, -50);
       set3 = true;
   }


}

