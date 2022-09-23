

// include servo library
#include <Servo.h> 

// declare servo pin
int servo_one_pin = 3;

// Create a servo object 
Servo servo_one;


void setup() {
  // define which pin servo is attached to
  servo_one.attach(servo_one_pin);

}

void loop() {
  // start servo at zero position
  servo_one.write(0);

  // 1 second delay before next command
  delay(1000);

  // move servo 90 degrees from zero position
  servo_one.write(90);

  delay(1000);

  // move servo 180 degrees from zero position
  servo_one.write(180);

  delay(1000);

  // move servo 270 degrees from zero position (note this is only a feature for Sunfonder motors)
  servo_one.write(270);

  // after this delay, the servos should start from 0 degrees and go through the cycle again
  delay(1000);
  
}
