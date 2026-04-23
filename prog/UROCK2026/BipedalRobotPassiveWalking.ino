#include <mcp_can.h>
#include <SPI.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// PIN DEFINITIONS (ESP32)
#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CS   5

MCP_CAN CAN0(SPI_CS);
Adafruit_MPU6050 mpu;

//ROBOT CONFIGURATION 
const int16_t BASE_WALK_RPM = 1500;  // Adjust for desired race speed [cite: 15]
const float KP_BALANCE = 120.0;     // Aggressiveness of balance correction
const float TARGET_PITCH = -2.5;    // Offset for center of mass (Calibration)

//STATE VARIABLES
int16_t current_encoder = 0;
float current_pitch = 0;

void setup() {
    Serial.begin(115200);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);

    // Init CAN
    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
        Serial.println("CAN Init OK!");
    } else { while (1); }
    CAN0.setMode(MCP_NORMAL);

    // Init IMU
    if (!mpu.begin()) {
        Serial.println("MPU6050 Failed!");
        while (1);
    }
    Serial.println("MPU6050 Ready!");
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // 1. Calculate Pitch (Simple Angle from Accelerometer)
    // For racing, you may want a Complementary Filter to reduce noise
    current_pitch = atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;

    // 2. Read Motor Feedback
    long unsigned int rxId;
    unsigned char len = 0;
    unsigned char rxBuf[8];

    if (CAN_MSGAVAIL == CAN0.checkReceive()) {
        CAN0.readMsgBuf(&rxId, &len, rxBuf);
        if (rxId == 0x201) {
            current_encoder = (rxBuf[0] << 8) | rxBuf[1]; // 0 - 8191
        }
    }

    // 3. PHASE-AWARE BALANCE LOGIC
    float pitch_error = current_pitch - TARGET_PITCH;
    
    // Weighting factor: Only correct balance when the crank is in a 'useful' position
    // We use a sine wave mapped to the encoder to smooth out the correction
    float phase_rad = ((float)current_encoder / 8191.0) * 2.0 * PI;
    float phase_weight = sin(phase_rad); 

    // 4. CALCULATE FINAL VELOCITY
    // If pitch_error is positive (falling forward), and phase_weight is positive (swinging),
    // then the correction increases the RPM.
    int16_t balance_correction = (int16_t)(pitch_error * KP_BALANCE * phase_weight);
    int16_t target_rpm = BASE_WALK_RPM + balance_correction;

    // 5. SEND COMMAND (Velocity Control)
    // Note: C620 ESCs usually take Current commands, so this is an inner-loop approximation
    sendMotorCurrent(target_rpm);

    delay(5); // 200Hz Control Loop
}

void sendMotorCurrent(int16_t current) {
    // Clamp values for safety [cite: 10]
    if (current > 10000) current = 10000;
    if (current < -10000) current = -10000;

    byte data[8];
    data[0] = current >> 8;
    data[1] = current & 0xFF;
    for (int i = 2; i < 8; i++) data[i] = 0;

    CAN0.sendMsgBuf(0x200, 0, 8, data);
}
