#include <main.h>
#include <ESP32Servo.h>

int counter = 0;
int counter_two = 1;

bool flag = true;

int defaultDelay = 250;

void setup() {
  Serial.begin(115200);


  
  // SETUP SERVO
  setupLeg();
  // readyLeg();
  // delay(3000);

  LEG1S1.write(75 + err[0][0]);
  LEG1S2.write(45 + err[0][1]);

  LEG2S1.write((75 * 270 / 180) + err[1][0]); //kana tambah 8
  LEG2S2.write((45 * 270 / 180) + err[1][1]); //kana tambah 8

  LEG3S1.write(75 + err[2][0]);
  LEG3S2.write(45 + err[2][1]);

  LEG4S1.write(75 + err[3][0]); //120
  LEG4S2.write(45 + err[3][1]); //75
  delay(3000);
}

void loop() {
  move2point();
  
  // counter++;
  // counter_two++;
  // delay(250);
}

void newL3(){
  LEG3S1.write(75 + err[2][0]);
  LEG3S2.write(45 + err[2][1]);
  delay(500);
  LEG3S1.write(68 + err[2][0]);
  LEG3S2.write(49 + err[2][1]);
  delay(500);
  LEG3S1.write(59 + err[2][0]);
  LEG3S2.write(24 + err[2][1]);
  delay(500);
  LEG3S1.write(64 + err[2][0]);
  LEG3S2.write(14 + err[2][1]);
  delay(500);
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

  delay(500);

  //---------------------------------------------------

  LEG1S1.write(70 + err[0][0]);
  LEG1S2.write(56 + err[0][1]);

  LEG2S1.write((64 * 270 / 180) + err[1][0]);
  LEG2S2.write((50 * 270 / 180) + err[1][1]);

  LEG3S1.write(68 + err[2][0]);
  LEG3S2.write(49 + err[2][1]);

  LEG4S1.write(71 + err[3][0]);
  LEG4S2.write(52 + err[3][1]);

  delay(500);
  
  //---------------------------------------------------

  LEG1S1.write(84 + err[0][0]);
  LEG1S2.write(82 + err[0][1]);

  LEG2S1.write((38 * 270 / 180) + err[1][0]);
  LEG2S2.write((36 * 270 / 180) + err[1][1]);

  LEG3S1.write(59 + err[2][0]);
  LEG3S2.write(24 + err[2][1]);

  LEG4S1.write(96 + err[3][0]);
  LEG4S2.write(61 + err[3][1]);

  delay(500);

  //---------------------------------------------------

  LEG1S1.write(93 + err[0][0]);
  LEG1S2.write(72 + err[0][1]);

  LEG2S1.write((48 * 270 / 180) + err[1][0]);
  LEG2S2.write((27 * 270 / 180) + err[1][1]);

  LEG3S1.write(64 + err[2][0]);
  LEG3S2.write(14 + err[2][1]);

  LEG4S1.write(106 + err[3][0]);
  LEG4S2.write(56 + err[3][1]);

  delay(500);
}

void move4point(){
  LEG1S1.write(75 + err[0][0]);
    LEG1S2.write(45 + err[0][1]);
    LEG3S1.write(75 + err[2][0]);
    LEG3S2.write(45 + err[2][1]);

    LEG2S1.write((48 * 270 / 180) + err[1][0]);
    LEG2S2.write((27 * 270 / 180) + err[1][1]);
    LEG4S1.write(106 + err[3][0]);
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
    LEG4S1.write(106 + err[3][0]);
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
    LEG4S1.write(106 + err[3][0]);
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