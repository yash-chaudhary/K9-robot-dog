
// include Servo library
#include <Servo.h>

// create instance of Servo class
Servo servo_one;
Servo servo_two;

// define servo pin
int servo_one_pin = 3;
int servo_two_pin = 5;

// define initial position
int pos = 0;

void setup() {
  // attach servo to corresponding pin
  // servo_one.attach(servo_one_pin);
  servo_two.attach(servo_two_pin);
  

}

void loop() {

  // move from 0 to 180 degrees
  for(pos; pos < 180; pos += 1) {
    // servo_one.write(pos);
    servo_two.write(pos);
    
    delay(10);
  }

  // move from 180 to 0 degrees
  for(pos; pos > 0; pos -= 1) {
    // servo_one.write(pos);
    servo_two.write(pos);
    delay(10);
  }
}
