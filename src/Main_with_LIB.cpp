
#include <arduino.h>
#include <ESP_Arduino_CAN.h>


ESP_Arduino_CAN Odrives(GPIO_NUM_4, GPIO_NUM_5);
void setup()
{
    Serial.begin(115200);
    vTaskDelay(1000);
    Odrives.RunStateCloseLoop(0);
    Odrives.Can_clear_rx_queue();
    Odrives.Can_clear_tx_queue();

    Odrives.setTrajaccel(0, 3, 3);

    Serial.println("SAVE_CONFIGU");
}

bool set1 = false;
bool set2 = false;
bool set3 = false;
bool set4 = false;

// void loop(){}


void loop()
{
     Odrives.update();
    Odrives.feedUpdate(11);

// Odrives.CanDebug();

// Serial.print(" ");
// Odrives.DebugMyData();


// if(Odrives.newupdate){
//     Odrives.CanDebug();
// }

//Serial.println(Odrives.newupdate);

Odrives.Debug(0);


   if (millis() > 1000 and set1 == false){

    Odrives.SetVelocity(0, 10);
       set1 = true;
   }

   if (millis() > 22000 and set4 == false){
    //Odrives.SetCtrlMode(0, 2, 1);
       Serial.println("CMD_____Vel//////////////////////////");
       Odrives.SetVelocity(0, 10);
       set4 = true;
   }

   if (millis() > 10000 and set2 == false){
       Serial.println("CMD_____TRAJ2//////////////////////////");

       Odrives.SetTrajPos (0, -250);
       set2 = true;
   }
    
    if (millis() > 20000 and set3 == false){
       Serial.println("CMD_____TRAJ2/////////////////////////////////");
       Odrives.SetTrajPos (0, -50);
       set3 = true;
   }


}

