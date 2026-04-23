#include <mcp_can.h>
#include <SPI.h>
#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CS   5

MCP_CAN CAN0(SPI_CS);

// SERVO CONTROL VARIABLES
// 0 to 8191 represents one full rotation of the rotor
int16_t target_angle = 4000;  // Change this to move the motor!
int16_t current_angle = 0;

// Tuning parameter: How aggressively it tries to reach the target
// If it oscillates/shakes, lower this. If it's too weak, raise it.
float Kp = 1.5; 

void setup() {
    Serial.begin(115200);
    
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);

    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN Initialized Successfully!");
    } else {
        Serial.println("CAN Init Failed.");
        while (1);
    }
    
    CAN0.setMode(MCP_NORMAL);
}

void loop() {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    if (CAN_MSGAVAIL == CAN0.checkReceive()) {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);

        if (rxId == 0x201) {
            current_angle = (rxBuf[0] << 8) | rxBuf[1];
            
            int16_t error = target_angle - current_angle;

            if (error > 4096) error -= 8192;
            if (error < -4096) error += 8192;

            int16_t output_current = error * Kp;

            //Clamp the maximum current so it doesn't spin dangerously fast
            if (output_current > 4000) output_current = 4000;
            if (output_current < -4000) output_current = -4000;

            byte data[8];
            data[0] = output_current >> 8;
            data[1] = output_current & 0xFF;
            
            for (int i = 2; i < 8; i++) data[i] = 0;

            CAN0.sendMsgBuf(0x200, 0, 8, data);
        }
    }
    delay(2); 
}
