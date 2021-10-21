#include <arduino.h>
#include <CAN.h>

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("CAN Sender");

    // start the CAN bus at 500 kbps
    if (!CAN.begin(250E3))
    {
        Serial.println("Starting CAN failed!");
        while (1)
            ;
    }
    Serial.println("CAN Sender OK");
}
// ID for ODrive
int canID = (0 << 5) + 0x00D;
// velocity set point

// CAN message

long lastmillis = 2000;
void loop()
{
    if (millis() > lastmillis)
    {
        float vel = 20;
        uint8_t stmp[6] = {0};
        memcpy(stmp, &vel, sizeof(vel));
        
        CAN.beginPacket(canID);
        CAN.write(stmp, 6);
        CAN.endPacket();
        Serial.println("send");
        lastmillis = millis() + 100;
    }
    // delay(1000);

    // try to parse packet
    int packetSize = CAN.parsePacket();

    if (packetSize)
    {
        // received a packet
        Serial.print("Received ");

        if (CAN.packetExtended())
        {
            Serial.print("extended ");
        }

        if (CAN.packetRtr())
        {
            // Remote transmission request, packet contains no data
            Serial.print("RTR ");
        }

        Serial.print("packet with id 0x");
        Serial.print(CAN.packetId(), HEX);

        if (CAN.packetRtr())
        {
            Serial.print(" and requested length ");
            Serial.println(CAN.packetDlc());
        }
        else
        {
            Serial.print(" and length ");
            Serial.println(packetSize);

            // only print packet data for non-RTR packets
            while (CAN.available())
            {
                Serial.print((char)CAN.read());
            }
            Serial.println();
        }

        Serial.println();
    }
}