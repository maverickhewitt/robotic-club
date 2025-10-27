#include "leg.h"

// Coordinate secondHalfLine() {
//    Coordinate pos;  // create a variable to store x and y

//    for (t = 0.5; t <= 1; t += delta_t) {
//     pos.x = x2 - t * step_length;  
//     pos.y = ground_offset;

//     Serial.print("Ground phase - x: ");
//     Serial.print(pos.x);
//     Serial.print(", y: ");
//     Serial.println(pos.y);

//     delay(50);
//   }





//   return pos;  // return the final x, y position

//   //move servo i and k
// }

void setup() {
  Serial.begin(115200);
  setupServo();

  // leg_i();
  // leg_k();
  // leg_l();
}

void loop() {
  settingServo();
  // move_IK();
  // move_JL();
}


// //REVERSE adjust angle
// void loop() {
//    float t;

//    //ikd how but ts works 

//    // 🟢 Air phase (foot up and forward)
//   for (t = 0; t <= 1; t += delta_t) {
//     x = bezierQuadratic(x2, x1, x0, t);
//     y = bezierQuadratic(y2, y1, y0, t);
  
//     Serial.print("Air phase - x: ");
//     Serial.print(x);
//     Serial.print(", y: ");
//     Serial.println(y);

//     IK_ThetaAngle(x, y);

//     //delay(200);
//   }

//   // 🔵 Ground phase (foot back and flat)
//   for (t = 0; t <= 1; t += delta_t) {
//     // move back linearly along the ground
//     x = x0 + t * step_length;  //REVERSE
//     y = ground_offset;
    

//     Serial.print("Ground phase - x: ");
//     Serial.print(x);
//     Serial.print(", y: ");
//     Serial.println(y);

//     IK_ThetaAngle(x, y);

//    //delay(200);
//   }
// }
 
