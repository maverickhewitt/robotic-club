#include <main.h>
#include <ESP32Servo.h>

int counter = 0;
int counter_two = 1;

bool flag = true;

int defaultDelay = 200;
int gyroDelay = 200;
int loopDelay = 10;

void setup() {
  Serial.begin(115200);

   // SETUP GYRO
  setupGyro();

  // SETUP SERVO
  setupLeg();
  // readyLeg();
  // delay(3000);

  // synchronizeAllServosStartAndWaitForAllServosToStop();
  // setSpeedForAllServos(300);

  LEG1S1.write(80 + err[0][0]);
  LEG1S2.write(40 + err[0][1]);

  LEG2S1.write((80 * 270 / 180) + err[1][0]); //kana tambah 8
  LEG2S2.write((40 * 270 / 180) + err[1][1]); //kana tambah 8

  LEG3S1.write(80 + err[2][0]);
  LEG3S2.write(40 + err[2][1]);

  LEG4S1.write(80 + err[3][0]); //120
  LEG4S2.write(40 + err[3][1]); //75
  delay(2000);

  updateGyro();

}

void loop() {
  updateGyro();

  if(counter == 4) counter = 1;
  if(counter_two == 4) counter_two = 1;

  int gyroRead = getAvgGyro();

  Serial.println(gyroRead);

  if (getAvgGyro() > 3) {
    // MOVEMENT TO THE LEFT
    // WORK WITH MECHA TO GET THE ANGLE
    newL1Move(2);//forward
    newL3Move(3); //backward
    newL2Move(3); //backward
    newL4Move(2);//foward
    delay(gyroDelay);
    newL1Move(3);//backward
    newL3Move(2); //foward
    newL2Move(2); //foward
    newL4Move(3);//backward
    delay(gyroDelay);
    Serial.println(gyroRead);

  
    } else if (getAvgGyro() < -3) {
    // MOVEMENT TO THE RIGHT
    // WORK WITH MECHA TO GET THE ANGLE
    newL1Move(3);//backward
    newL3Move(3); //backward
    newL2Move(2); //forward
    newL4Move(2);//forward
    delay(gyroDelay);
    newL1Move(2);//foward
    newL3Move(2); //foward
    newL2Move(3); //backward
    newL4Move(3);//backward
    delay(gyroDelay);
    Serial.println(gyroRead);

    } else {
        newL1Move(1);//raise
        newL3Move(1); //raise

        delay(defaultDelay);

        newL2Move(3); //backward
        newL4Move(3);//backward 

        delay(defaultDelay);

        newL1Move(2); //foward
        newL3Move(2); //foward

        delay(defaultDelay);

        newL2Move(1); //raise
        newL4Move(1); //raise

        delay(180);

        newL1Move(3); //backward
        newL3Move(3); //backward

        delay(defaultDelay);

        newL2Move(2); //foward
        newL4Move(2); //foward
        
        delay(180);
        Serial.println(gyroRead);
    }

/*
  // move3pointNewMantap();
    LEG2S1.write((70 * 270 / 180) + err[1][0], 300);
    LEG2S2.write((46 * 270 / 180) + err[1][1], 300);

    synchronizeAllServosStartAndWaitForAllServosToStop();

    //LEG 4 RAISE
    LEG4S1.write(70 + err[3][0], 300);
    LEG4S2.write(46 + err[3][1], 300);

    // delay(defaultDelay);
    synchronizeAllServosStartAndWaitForAllServosToStop();


    //LEG 1 BACKWARD
    LEG1S1.write(75 + err[0][0], 300);
    LEG1S2.write(26 + err[0][1], 300);

    synchronizeAllServosStartAndWaitForAllServosToStop();

    //leg 3 BACKWARD
    LEG3S1.write(88 + err[2][0], 300);
    LEG3S2.write(49 + err[2][1], 300);

    // delay(defaultDelay);
    synchronizeAllServosStartAndWaitForAllServosToStop();


    //---------------------------------------------------

    //leg 2 FORWARD
    LEG2S1.write((66 * 270 / 180) + err[1][0], 300);
    LEG2S2.write((25 * 270 / 180) + err[1][1], 300);

    synchronizeAllServosStartAndWaitForAllServosToStop();

    //LEG 4 FORWARD 
    LEG4S1.write(94 + err[3][0], 300);
    LEG4S2.write(46 + err[3][1], 300);

    // delay(defaultDelay);
    synchronizeAllServosStartAndWaitForAllServosToStop();

    //LEG 1 RAISE
    LEG1S1.write(73 + err[0][0], 300);
    LEG1S2.write(49 + err[0][1], 300);

    synchronizeAllServosStartAndWaitForAllServosToStop();

    //leg 3 RAISE
    LEG3S1.write(73 + err[2][0], 300);
    LEG3S2.write(49 + err[2][1], 300);

    // delay(defaultDelay);
    synchronizeAllServosStartAndWaitForAllServosToStop();
    
    //---------------------------------------------------
    //leg 2 BACKWARD
    LEG2S1.write((93 * 270 / 180) + err[1][0], 300);
    LEG2S2.write((44 * 270 / 180) + err[1][1], 300);

    synchronizeAllServosStartAndWaitForAllServosToStop();

    //LEG 4 BACKWARD
    LEG4S1.write(67 + err[3][0], 300);
    LEG4S2.write(32 + err[3][1], 300);

    // delay(defaultDelay);
    synchronizeAllServosStartAndWaitForAllServosToStop();

    //LEG 1 FOWARD
    LEG1S1.write(94 + err[0][0], 300);
    LEG1S2.write(53 + err[0][1], 300);

    synchronizeAllServosStartAndWaitForAllServosToStop();

    //LEG 3 FORWARD
    LEG3S1.write(73 + err[2][0], 300);
    LEG3S2.write(24 + err[2][1], 300);

    // delay(defaultDelay);  
    synchronizeAllServosStartAndWaitForAllServosToStop();

    */
    }

void newL3(){
  LEG3S1.write(75 + err[2][0]);
  LEG3S2.write(45 + err[2][1]);
  delay(300);
  LEG3S1.write(68 + err[2][0]);
  LEG3S2.write(49 + err[2][1]);
  delay(300);
  LEG3S1.write(59 + err[2][0]);
  LEG3S2.write(24 + err[2][1]);
  delay(300);
  LEG3S1.write(64 + err[2][0]);
  LEG3S2.write(14 + err[2][1]);
  delay(300);
}

void newMove(){
    LEG1S1.write(75 + err[0][0]);
  LEG1S2.write(45 + err[0][1]);

  LEG2S1.write((75 * 270 / 180) + err[1][0]);
  LEG2S2.write((45 * 270 / 180) + err[1][1]);

  LEG3S1.write(75 + err[2][0]);
  LEG3S2.write(45 + err[2][1]);

  LEG4S1.write(75 + err[3][0]);
  LEG4S2.write(45 + err[3][1]);

  delay(300);

  //---------------------------------------------------

  LEG1S1.write(70 + err[0][0]);
  LEG1S2.write(56 + err[0][1]);

  LEG2S1.write((64 * 270 / 180) + err[1][0]);
  LEG2S2.write((50 * 270 / 180) + err[1][1]);

  LEG3S1.write(68 + err[2][0]);
  LEG3S2.write(49 + err[2][1]);

  LEG4S1.write(71 + err[3][0]);
  LEG4S2.write(52 + err[3][1]);

  delay(300);
  
  //---------------------------------------------------

  LEG1S1.write(84 + err[0][0]);
  LEG1S2.write(82 + err[0][1]);

  LEG2S1.write((38 * 270 / 180) + err[1][0]);
  LEG2S2.write((36 * 270 / 180) + err[1][1]);

  LEG3S1.write(59 + err[2][0]);
  LEG3S2.write(24 + err[2][1]);

  LEG4S1.write(96 + err[3][0]);
  LEG4S2.write(61 + err[3][1]);

  delay(300);

  //---------------------------------------------------

  LEG1S1.write(93 + err[0][0]);
  LEG1S2.write(72 + err[0][1]);

  LEG2S1.write((48 * 270 / 180) + err[1][0]);
  LEG2S2.write((27 * 270 / 180) + err[1][1]);

  LEG3S1.write(64 + err[2][0]);
  LEG3S2.write(14 + err[2][1]);

  LEG4S1.write(3006 + err[3][0]);
  LEG4S2.write(56 + err[3][1]);

  delay(300);
}

void move4point(){
  LEG1S1.write(75 + err[0][0]);
    LEG1S2.write(45 + err[0][1]);
    LEG3S1.write(75 + err[2][0]);
    LEG3S2.write(45 + err[2][1]);

    LEG2S1.write((48 * 270 / 180) + err[1][0]);
    LEG2S2.write((27 * 270 / 180) + err[1][1]);
    LEG4S1.write(3006 + err[3][0]);
    LEG4S2.write(56 + err[3][1]);

    delay(defaultDelay);

    //---------------------------------------------------

    LEG1S1.write(70 + err[0][0]);
    LEG1S2.write(56 + err[0][1]);
    LEG3S1.write(68 + err[2][0]);
    LEG3S2.write(49 + err[2][1]);

    LEG2S1.write((75 * 270 / 180) + err[1][0]);
    LEG2S2.write((45 * 270 / 180) + err[1][1]);
    LEG4S1.write(75 + err[3][0]);
    LEG4S2.write(45 + err[3][1]);

    delay(defaultDelay);
    
    //---------------------------------------------------

    LEG1S1.write(84 + err[0][0]);
    LEG1S2.write(82 + err[0][1]);
    LEG3S1.write(59 + err[2][0]);
    LEG3S2.write(24 + err[2][1]);

    LEG2S1.write((64 * 270 / 180) + err[1][0]);
    LEG2S2.write((50 * 270 / 180) + err[1][1]);
    LEG4S1.write(71 + err[3][0]);
    LEG4S2.write(52 + err[3][1]);

    delay(defaultDelay);

    //---------------------------------------------------

    LEG1S1.write(93 + err[0][0]);
    LEG1S2.write(72 + err[0][1]);
    LEG3S1.write(64 + err[2][0]);
    LEG3S2.write(14 + err[2][1]);

    LEG2S1.write((38 * 270 / 180) + err[1][0]);
    LEG2S2.write((36 * 270 / 180) + err[1][1]);
    LEG4S1.write(96 + err[3][0]);
    LEG4S2.write(61 + err[3][1]);

    delay(defaultDelay);  
}

void move3point(){
  LEG1S1.write(75 + err[0][0]);
    LEG1S2.write(45 + err[0][1]);
    LEG3S1.write(75 + err[2][0]);
    LEG3S2.write(45 + err[2][1]);

    LEG2S1.write((48 * 270 / 180) + err[1][0]);
    LEG2S2.write((27 * 270 / 180) + err[1][1]);
    LEG4S1.write(3006 + err[3][0]);
    LEG4S2.write(56 + err[3][1]);
    
    delay(defaultDelay);

    //---------------------------------------------------

    LEG1S1.write(62 + err[0][0]);
    LEG1S2.write(34 + err[0][1]);
    LEG3S1.write(68 + err[2][0]);
    LEG3S2.write(49 + err[2][1]);

    LEG2S1.write((75 * 270 / 180) + err[1][0]);
    LEG2S2.write((45 * 270 / 180) + err[1][1]);
    LEG4S1.write(75 + err[3][0]);
    LEG4S2.write(45 + err[3][1]);

    delay(defaultDelay);
    
    //---------------------------------------------------

    LEG1S1.write(93 + err[0][0]);
    LEG1S2.write(72 + err[0][1]);
    LEG3S1.write(64 + err[2][0]);
    LEG3S2.write(14 + err[2][1]);

    LEG2S1.write((64 * 270 / 180) + err[1][0]);
    LEG2S2.write((50 * 270 / 180) + err[1][1]);
    LEG4S1.write(71 + err[3][0]);
    LEG4S2.write(52 + err[3][1]);

    delay(defaultDelay);  
}

void move2point(){
   LEG1S1.write(75 + err[0][0]);
    LEG1S2.write(45 + err[0][1]);
    LEG3S1.write(75 + err[2][0]);
    LEG3S2.write(45 + err[2][1]);
  
     LEG2S1.write((48 * 270 / 180) + err[1][0]);
    LEG2S2.write((27 * 270 / 180) + err[1][1]);
    LEG4S1.write(3006 + err[3][0]);
    LEG4S2.write(56 + err[3][1]);


    delay(defaultDelay);
    
    //---------------------------------------------------
    //leg 1
    LEG1S1.write(93 + err[0][0]);
    LEG1S2.write(72 + err[0][1]);
    LEG3S1.write(64 + err[2][0]);
    LEG3S2.write(14 + err[2][1]);

    //leg 2
   LEG2S1.write((75 * 270 / 180) + err[1][0]);
    LEG2S2.write((45 * 270 / 180) + err[1][1]);
    LEG4S1.write(75 + err[3][0]);
    LEG4S2.write(45 + err[3][1]);

    delay(defaultDelay);  

}

void move3pointNewAngle(){
   //leg 2 RAISE
    LEG2S1.write((70 * 270 / 180) + err[1][0]);
    LEG2S2.write((46 * 270 / 180) + err[1][1]);

    //LEG 4 RAISE
    LEG4S1.write(70 + err[3][0]);
    LEG4S2.write(46 + err[3][1]);

    //LEG 1 BACKWARD
    LEG1S1.write(75 + err[0][0]);
    LEG1S2.write(26 + err[0][1]);

    //leg 3 BACKWARD
    LEG3S1.write(88 + err[2][0]);
    LEG3S2.write(49 + err[2][1]);

    delay(defaultDelay);

    //---------------------------------------------------

    //leg 2 FORWARD
    LEG2S1.write((66 * 270 / 180) + err[1][0]);
    LEG2S2.write((25 * 270 / 180) + err[1][1]);

    //LEG 4 FORWARD 
    LEG4S1.write(94 + err[3][0]);
    LEG4S2.write(46 + err[3][1]);

    //LEG 1 RAISE
    LEG1S1.write(73 + err[0][0]);
    LEG1S2.write(49 + err[0][1]);

    //leg 3 RAISE
    LEG3S1.write(73 + err[2][0]);
    LEG3S2.write(49 + err[2][1]);

    delay(defaultDelay);
    
    //---------------------------------------------------
    //leg 2 BACKWARD
    LEG2S1.write((93 * 270 / 180) + err[1][0]);
    LEG2S2.write((44 * 270 / 180) + err[1][1]);

    //LEG 4 BACKWARD
    LEG4S1.write(67 + err[3][0]);
    LEG4S2.write(32 + err[3][1]);

    //LEG 1 FOWARD
    LEG1S1.write(94 + err[0][0]);
    LEG1S2.write(53 + err[0][1]);

    //LEG 3 FORWARD
    LEG3S1.write(73 + err[2][0]);
    LEG3S2.write(24 + err[2][1]);

    delay(defaultDelay);  
}

void move3pointNewMantap(){
      //leg 2 RAISE -- > leg 4
    LEG2S1.write((70 * 270 / 180) + err[1][0]);
    LEG2S2.write((46 * 270 / 180) + err[1][1]);

    //LEG 4 RAISE -- leg 2
    LEG4S1.write(70 + err[3][0]);
    LEG4S2.write(46 + err[3][1]);

    delay(defaultDelay);

    //LEG 1 BACKWARD -- leg 3
    LEG1S1.write(75 + err[0][0]);
    LEG1S2.write(26 + err[0][1]);

    //leg 3 BACKWARD -- leg 1
    LEG3S1.write(88 + err[2][0]);
    LEG3S2.write(49 + err[2][1]);

    delay(defaultDelay);

    //---------------------------------------------------

    //leg 2 FORWARD
    LEG2S1.write((66 * 270 / 180) + err[1][0]);
    LEG2S2.write((25 * 270 / 180) + err[1][1]);

    //LEG 4 FORWARD 
    LEG4S1.write(94 + err[3][0]);
    LEG4S2.write(46 + err[3][1]);

    delay(defaultDelay);

    //LEG 1 RAISE
    LEG1S1.write(73 + err[0][0]);
    LEG1S2.write(49 + err[0][1]);

    //leg 3 RAISE
    LEG3S1.write(73 + err[2][0]);
    LEG3S2.write(49 + err[2][1]);

    delay(defaultDelay);
    
    //---------------------------------------------------
    //leg 2 BACKWARD
    LEG2S1.write((93 * 270 / 180) + err[1][0]);
    LEG2S2.write((44 * 270 / 180) + err[1][1]);

    //LEG 4 BACKWARD
    LEG4S1.write(67 + err[3][0]);
    LEG4S2.write(32 + err[3][1]);

    delay(defaultDelay);

    //LEG 1 FOWARD
    LEG1S1.write(94 + err[0][0]);
    LEG1S2.write(53 + err[0][1]);

    //LEG 3 FORWARD
    LEG3S1.write(73 + err[2][0]);
    LEG3S2.write(24 + err[2][1]);

    delay(defaultDelay);  
}

void adalower(){
  //Raise Leg 2 and Leg 4
LEG2S1.write((70 * 270 / 180) + err[1][0]); // Raise Leg 2
LEG2S2.write((46 * 270 / 180) + err[1][1]);
LEG4S1.write(70 + err[3][0]);              // Raise Leg 4
LEG4S2.write(46 + err[3][1]);
delay(defaultDelay);

// Move Leg 1 and Leg 3 Backward
LEG1S1.write(75 + err[0][0]);             // Move Leg 1 Backward
LEG1S2.write(26 + err[0][1]);
LEG3S1.write(88 + err[2][0]);             // Move Leg 3 Backward
LEG3S2.write(49 + err[2][1]);
delay(defaultDelay);

// Lower Leg 2 and Leg 4
LEG2S1.write((66 * 270 / 180) + err[1][0]); // Lower Leg 2 to ground
LEG2S2.write((25 * 270 / 180) + err[1][1]);
LEG4S1.write(94 + err[3][0]);              // Lower Leg 4 to ground
LEG4S2.write(46 + err[3][1]);
delay(defaultDelay);

// Raise Leg 1 and Leg 3
LEG1S1.write(73 + err[0][0]);             // Raise Leg 1
LEG1S2.write(49 + err[0][1]);
LEG3S1.write(73 + err[2][0]);             // Raise Leg 3
LEG3S2.write(49 + err[2][1]);
delay(defaultDelay);

// Move Leg 2 and Leg 4 Backward
LEG2S1.write((93 * 270 / 180) + err[1][0]); // Move Leg 2 Backward
LEG2S2.write((44 * 270 / 180) + err[1][1]);
LEG4S1.write(67 + err[3][0]);              // Move Leg 4 Backward
LEG4S2.write(32 + err[3][1]);
delay(defaultDelay);

// Lower Leg 1 and Leg 3, Move Forward
LEG1S1.write(94 + err[0][0]);             // Lower Leg 1 and move forward
LEG1S2.write(53 + err[0][1]);
LEG3S1.write(73 + err[2][0]);             // Lower Leg 3 and move forward
LEG3S2.write(24 + err[2][1]);
delay(defaultDelay);

}

void leg2FL() {
  // Phase 1: Raise Leg 2 (Both servos move together)
  for (int pos = 70; pos >= 66; pos--) {
    LEG2S1.write((pos * 270 / 180) + err[1][0]); // Lift LEG2S1
    LEG2S2.write((46 - (70 - pos)) * 270 / 180 + err[1][1]); // Adjust LEG2S2 simultaneously
    delay(loopDelay);
  }

  // Phase 2: Move Leg 2 Forward
  for (int pos = 66; pos <= 93; pos++) {
    LEG2S1.write((pos * 270 / 180) + err[1][0]); // Move LEG2S1 forward
    LEG2S2.write((25 + (pos - 66)) * 270 / 180 + err[1][1]); // Sync LEG2S2
    delay(loopDelay);
  }

  // Phase 3: Lower Leg 2 (Both servos move together)
  for (int pos = 93; pos >= 70; pos--) {
    LEG2S1.write((pos * 270 / 180) + err[1][0]); // Lower LEG2S1
    LEG2S2.write((44 - (93 - pos)) * 270 / 180 + err[1][1]); // Sync LEG2S2
    delay(loopDelay);
  }
}


void leg1FL(){
  for(int leg1S1 = 75; leg1S1 >= 73; leg1S1--){
    LEG1S1.write((leg1S1) + err[0][0]);
    delay(loopDelay);
  }
  for(int leg1S2 = 26; leg1S2 <= 49; leg1S2++){
    LEG1S2.write((leg1S2) + err[0][1]);
    delay(loopDelay);
  }

  for(int leg1S1 = 73; leg1S1 <= 94; leg1S1++){
    LEG1S1.write((leg1S1) + err[0][0]);
    delay(loopDelay);
  }
  for(int leg1S2 = 49; leg1S2 <= 53; leg1S2++){
    LEG1S2.write((leg1S2) + err[0][1]);
    delay(loopDelay);
  }

  for(int leg1S1 = 94; leg1S1 >= 75; leg1S1--){
    LEG1S1.write((leg1S1) + err[0][0]);
    delay(loopDelay);
  }
  for(int leg1S2 = 53; leg1S2 >= 26; leg1S2--){
    LEG1S2.write((leg1S2) + err[0][1]);
    delay(loopDelay);
  }
}

void leg3FL(){
  for(int leg3S1 = 88; leg3S1 >= 73; leg3S1--){
    LEG3S1.write(leg3S1 + err[2][0]);
    delay(loopDelay);
  }
  for(int leg3S2 = 49; leg3S2 >= 49; leg3S2--){
    LEG3S2.write(leg3S2 + err[2][1]);
    delay(loopDelay);
  }

  for(int leg3S1 = 73; leg3S1 <= 73; leg3S1++){
    LEG3S1.write(leg3S1 + err[2][0]);
    delay(loopDelay);
  }
  for(int leg3S2 = 49; leg3S2 >= 24; leg3S2--){
    LEG3S2.write(leg3S2 + err[2][1]);
    delay(loopDelay);
  }

  for(int leg3S1 = 73; leg3S1 >= 88; leg3S1++){
    LEG3S1.write(leg3S1 + err[2][0]);
    delay(loopDelay);
  }
  for(int leg3S2 = 24; leg3S2 <= 49; leg3S2++){
    LEG3S2.write(leg3S2 + err[2][1]);
    delay(loopDelay);
  }
}

void leg4FL(){
  for(int leg4S1 = 70; leg4S1 <= 94; leg4S1++){
    LEG4S1.write(leg4S1 + err[3][0]);
    delay(loopDelay);
  }
  for(int leg4S2 = 46; leg4S2 >= 46; leg4S2--){
    LEG4S2.write(leg4S2 + err[3][1]);
    delay(loopDelay);
  }

  for(int leg4S1 = 90; leg4S1 >= 67; leg4S1--){
    LEG4S1.write(leg4S1 + err[3][0]);
    delay(loopDelay);
  }
  for(int leg4S2 = 46; leg4S2 >= 32; leg4S2--){
    LEG4S2.write(leg4S2 + err[3][1]);
    delay(loopDelay);
  }

  for(int leg4S1 = 67; leg4S1 <= 70; leg4S1++){
    LEG4S1.write(leg4S1 + err[3][0]);
    delay(loopDelay);
  }
  for(int leg4S2 = 32; leg4S2 <= 46; leg4S2++){
    LEG4S2.write(leg4S2 + err[3][1]);
    delay(loopDelay);
  }
}

void semuaFL(){
  
}



