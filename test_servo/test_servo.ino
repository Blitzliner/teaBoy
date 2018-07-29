#include <SoftwareServo.h>

SoftwareServo servo;
int pos = 0;  

void setup()
{
  servo.attach(1);
  servo.setMaximumPulse(2100);
  servo.setMinimumPulse(550);
}

void loop()
{
  /*
  for(pos = 0; pos < 180; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    servo.write(pos);              // tell servo to go to position in variable 'pos' 
    SoftwareServo::refresh();        // must call at least once every 50ms or so to keep your servos updating
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=1; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    servo.write(pos);              // tell servo to go to position in variable 'pos' 
    SoftwareServo::refresh();
    delay(15);                       // waits 15ms for the servo to reach the position 
  }*/    
  //delay(500);
  servo.write(0);              
  for(int i=0; i<50; i++) {
    SoftwareServo::refresh();  
    delay(20); 
  }
  delay(2000);
  
  servo.write(180);              
  for(int i=0; i<50; i++) {
    SoftwareServo::refresh();  
    delay(20); 
  }
  
  delay(2000);
}

