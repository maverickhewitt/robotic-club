#define IR_PIN 27

int sensor;

void senseRamp(){
  sensor = digitalRead(IR_PIN);
  if(sensor == 1 ){
    //pass
  }else{
    //pass
  }
}