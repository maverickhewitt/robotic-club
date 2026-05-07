#include <mcp_can.h>
#include <SPI.h>

#define SPI_SCK  18
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CS    5

#define TICKS_PER_REV 8192.0
#define GEAR_RATIO 19.2032  // The exact reduction ratio for the M3508 P19
#define MAX_OUTPUT 16000

MCP_CAN CAN0(SPI_CS);

// ================= STATE & CONTROL =================

// Global targets that update via Serial Monitor
float targetAngle1 = 0.0;
float targetAngle4 = 0.0;

// Global PID variables so they can be updated via Serial Monitor
float Kp = 300.0; 
float Ki = 0.0; 
float Kd = 30.0; 

uint16_t raw1, raw4, prevRaw1, prevRaw4;
int32_t ticks1 = 0, ticks4 = 0;
float angle1 = 0, angle4 = 0;

float integral1 = 0, prevErr1 = 0;
float integral4 = 0, prevErr4 = 0;

unsigned long lastLoop = 0;

// ================= FEEDBACK =================

void readCAN() {
  unsigned long id;
  unsigned char len;
  unsigned char buf[8];

  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    CAN0.readMsgBuf(&id, &len, buf);

    if (id == 0x201) {
      raw1 = (buf[0] << 8) | buf[1];
      int d = raw1 - prevRaw1;
      if (d > 4096) d -= 8192;
      if (d < -4096) d += 8192;
      ticks1 += d;
      prevRaw1 = raw1;
    }
    if (id == 0x204) {
      raw4 = (buf[0] << 8) | buf[1];
      int d = raw4 - prevRaw4;
      if (d > 4096) d -= 8192;
      if (d < -4096) d += 8192;
      ticks4 += d;
      prevRaw4 = raw4;
    }
  }
  
  // Output shaft position math
  angle1 = (ticks1 * 360.0) / (TICKS_PER_REV * GEAR_RATIO);
  angle4 = (ticks4 * 360.0) / (TICKS_PER_REV * GEAR_RATIO);
}

void send(int16_t m1, int16_t m4) {
  byte d[8] = {0};
  d[0] = m1 >> 8; d[1] = m1 & 0xFF;
  d[6] = m4 >> 8; d[7] = m4 & 0xFF;
  CAN0.sendMsgBuf(0x200, 0, 8, d);
}

int16_t PID(float target, float current, float &integral, float &prevErr, float dt) {
  // Kp, Ki, and Kd are now pulled from the global variables above!
  float err = target - current;
  integral += err * dt;
  float deriv = (err - prevErr) / dt;
  prevErr = err;
  return constrain((Kp * err) + (Ki * integral) + (Kd * deriv), -MAX_OUTPUT, MAX_OUTPUT);
}

// ================= MAIN LOOP =================

void setup() {
  Serial.begin(115200);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN OK");
  } else {
    while (1) Serial.println("CAN FAIL");
  }
  CAN0.setMode(MCP_NORMAL);
  
  Serial.println("System Ready.");
  Serial.println("--- MOVEMENT COMMANDS ---");
  Serial.println("a[angle] -> Motor 1 (e.g., a90)");
  Serial.println("b[angle] -> Motor 4 (e.g., b-45)");
  Serial.println("c[angle] -> Both    (e.g., c180)");
  Serial.println("--- TUNING COMMANDS ---");
  Serial.println("p[value] -> Set Kp  (e.g., p500)");
  Serial.println("i[value] -> Set Ki  (e.g., i1.5)");
  Serial.println("d[value] -> Set Kd  (e.g., d50)");
  
  lastLoop = micros();
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastLoop) / 1e6;
  lastLoop = now;
  if (dt <= 0) dt = 0.001;

  readCAN();

  // --- READ SERIAL INPUT ---
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read until enter is pressed
    input.trim(); // Remove any extra spaces or invisible characters

    if (input.length() > 0) {
      char command = input.charAt(0); // Get the first letter 
      float value = input.substring(1).toFloat(); // Convert the rest to a number

      // Angle Commands
      if (command == 'a' || command == 'A') {
        targetAngle1 = value;
        Serial.print("--> Motor 1 target set to: "); Serial.println(targetAngle1);
      } 
      else if (command == 'b' || command == 'B') {
        targetAngle4 = value;
        Serial.print("--> Motor 4 target set to: "); Serial.println(targetAngle4);
      } 
      else if (command == 'c' || command == 'C') {
        targetAngle1 = value;
        targetAngle4 = value;
        Serial.print("--> Both motors target set to: "); Serial.println(value);
      } 
      // PID Tuning Commands
      else if (command == 'p' || command == 'P') {
        Kp = value;
        Serial.print("--> Kp updated to: "); Serial.println(Kp);
      }
      else if (command == 'i' || command == 'I') {
        Ki = value;
        
        // Pro-tip: When changing Ki live, it's best to reset the accumulated integral
        // so the motor doesn't violently jerk from old built-up errors.
        integral1 = 0; 
        integral4 = 0; 
        
        Serial.print("--> Ki updated to: "); Serial.println(Ki);
      }
      else if (command == 'd' || command == 'D') {
        Kd = value;
        Serial.print("--> Kd updated to: "); Serial.println(Kd);
      }
      else {
        Serial.println("Invalid command.");
      }
    }
  }

  // --- RUN MOTORS ---
  int16_t out1 = PID(targetAngle1, angle1, integral1, prevErr1, dt);
  int16_t out4 = PID(targetAngle4, angle4, integral4, prevErr4, dt);
  send(out1, out4);
  
  // --- DEBUG OUTPUT ---
  static long lastPrint = 0;
  if (millis() - lastPrint > 250) {
    Serial.print("M1: "); Serial.print(angle1);
    Serial.print(" | M4: "); Serial.println(angle4);
    lastPrint = millis();
  }
}