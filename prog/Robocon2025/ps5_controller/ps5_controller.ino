#include <Arduino.h>
#include <ps5Controller.h>

void setup() {
  Serial.begin(115200);
  ps5.begin("64:17:CD:54:0D:AF"); //ganti dengan ps5 mac address
}

void loop() {
  if (ps5.isConnected() == true){
     Serial.println("connected");
     delay(3000);
  }
    while (ps5.isConnected() == true) 
    {      
      if(ps5.Up())
      {
        Serial.println("up");
        DJI_Increase_Speed += 1;
      } 

       if(ps5.Down())
      {
        Serial.println("down");
        DJI_Increase_Speed -= 1;
      } 
      Analog_Stick_Calc ( float(ps5.LStickX()), float(ps5.LStickY()), &Magnitude_L, &Angle_L);
       Serial.print("magnitude now:");
	     Serial.println(Magnitude_L);
	     Serial.print("angle now:");
	     Serial.println(Angle_L);
      
	     delay(100);
      
      if(Angle_L>60 && Angle_L<=120)
      { //foward

      }

      if(Angle_L>-90 && Angle_L <=-60 || Angle_L >-120 && Angle_L <=-90 )
      { //backward

      }

      if(Angle_L> 0 && Angle_L<= 30 || Angle_L>-30 && Angle_L<0 )
      { //rightward

      }

      if(Angle_L>150 && Angle_L <=180 || Angle_L>-180 && Angle_L <=-150 )
      { //left

      }
       else if(Angle_L==0)
      {
      }
      else if(Angle_L>30 && Angle_L <=60)
	    {
		     //UPWARD_RIGHT;
	    }
      else if(Angle_L>120 && Angle_L<=150)
	    {
		     //  UPWARD_LEFT;
	    }
      else if(Angle_L>-60 && Angle_L<=-30)
	    {
		     //  Loward_RIGHT;
	    }
      else if(Angle_L-150 && Angle_L <=-120)
	    {
		     //LOWARD_LEFT;
	    }

      if(ps5.R1()){

      }
     if(ps5.L1()){

      }  
    }
  }
  
