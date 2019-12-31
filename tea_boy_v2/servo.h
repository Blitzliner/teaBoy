#include <SoftwareServo.h>

#define SERVO_MIN               45
#define SERVO_MAX               135


int _pin_servo;
SoftwareServo servo;

void init_servo(int pin) {
  _pin_servo = pin;
  
  pinMode(_pin_servo, OUTPUT);
  
  servo.attach(_pin_servo);
  servo.setMaximumPulse(2100);
  servo.setMinimumPulse(550);
  servo.write(SERVO_MAX);

  for (int i=0; i<600/20; i++) { /* stellzeit */
    SoftwareServo::refresh();  
    delay(20); 
  }
}


void set_servo(uint8_t degree, uint32_t duration) {
  static int last_angle = SERVO_MAX; /* for servo */
  const int diff = abs(degree - last_angle);
  const int wait = (float)duration / (float)diff * 0.5 * 1000.0;
  
  if (last_angle < degree) {
    for (float k = last_angle; k < degree; k+=0.5) {
      servo.write(k); 
      delayMicroseconds(wait);
      SoftwareServo::refresh(); 
    }
  } else if (last_angle > degree) {
    for (float k = last_angle; k > degree; k-=0.5) {
      servo.write(k);
      delayMicroseconds(wait); 
      SoftwareServo::refresh(); 
    }
  }
  last_angle = degree;
}
