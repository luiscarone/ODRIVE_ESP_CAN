#include <CAN.h>
#include <arduino.h>


void setup() {
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("CAN Sender");

    // start the CAN bus at 500 kbps
    if (!CAN.begin(500E3)) {
        Serial.println("Starting CAN failed!");
        while (1)
            ;
    }
}
// ID for ODrive
int canID = (0x003 << 5) + 0x00D;
// velocity set point
int setV = 20;
// CAN message
unsigned char stmp[6] = {setV, (setV >> 8), (setV >> 16), (setV >> 24), 0, 0};

void loop() {
    CAN.beginPacket(canID);
    CAN.write(stmp, 6);
    CAN.endPacket();

    delay(5000);

    // try to parse packet
    int packetSize = CAN.parsePacket();

    if (packetSize) {
        // received a packet
        Serial.print("Received ");

        if (CAN.packetExtended()) {
            Serial.print("extended ");
        }

        if (CAN.packetRtr()) {
            // Remote transmission request, packet contains no data
            Serial.print("RTR ");
        }

        Serial.print("packet with id 0x");
        Serial.print(CAN.packetId(), HEX);

        if (CAN.packetRtr()) {
            Serial.print(" and requested length ");
            Serial.println(CAN.packetDlc());
        } else {
            Serial.print(" and length ");
            Serial.println(packetSize);

            // only print packet data for non-RTR packets
            while (CAN.available()) {
                Serial.print((char)CAN.read());
            }
            Serial.println();
        }
        delay(5000);
        Serial.println();
    }
}