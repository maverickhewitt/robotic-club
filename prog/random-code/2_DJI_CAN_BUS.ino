#include <mcp_can.h>
#include <SPI.h>

#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CS   5

MCP_CAN CAN0(SPI_CS);

// MOTOR 1 VARIABLES
int16_t target_1 = 4000;
int16_t current_1 = 0;
int16_t out_1 = 0;

// MOTOR 2 VARIABLES (New)
int16_t target_2 = 1000; // Different target for testing
int16_t current_2 = 0;
int16_t out_2 = 0;

float Kp = 1.5; 

void setup() {
    Serial.begin(115200);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);

    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN Initialized!");
    } else {
        Serial.println("CAN Failed.");
        while (1);
    }
    CAN0.setMode(MCP_NORMAL);
}

void loop() {
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    // 1. READ FEEDBACK FROM MOTORS
    if (CAN_MSGAVAIL == CAN0.checkReceive()) {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);

        if (rxId == 0x201) { // Feedback from Motor 1
            current_1 = (rxBuf[0] << 8) | rxBuf[1];
        } 
        else if (rxId == 0x202) { // Feedback from Motor 2
            current_2 = (rxBuf[0] << 8) | rxBuf[1];
        }
    }

    // 2. CALCULATE P-CONTROL FOR BOTH
    out_1 = calculatePID(target_1, current_1);
    out_2 = calculatePID(target_2, current_2);

    // 3. SEND ONE MESSAGE TO RULE THEM BOTH
    byte data[8];
    
    // Motor 1 Slot (Bytes 0, 1)
    data[0] = out_1 >> 8;
    data[1] = out_1 & 0xFF;
    
    // Motor 2 Slot (Bytes 2, 3)
    data[2] = out_2 >> 8;
    data[3] = out_2 & 0xFF;
    
    // Motor 3 & 4 (Bytes 4-7) - Keep at 0 if not used
    data[4] = 0; data[5] = 0;
    data[6] = 0; data[7] = 0;

    CAN0.sendMsgBuf(0x200, 0, 8, data);

    delay(2); 
}

// Helper function to keep loop() clean
int16_t calculatePID(int16_t target, int16_t current) {
    int16_t error = target - current;
    if (error > 4096) error -= 8192;
    if (error < -4096) error += 8192;
    
    int16_t output = error * Kp;
    if (output > 4000) output = 4000;
    if (output < -4000) output = -4000;
    return output;
}
