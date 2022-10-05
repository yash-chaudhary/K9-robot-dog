
/*  
 *  Project Name: K9
 *  Author: Yash Chaudhary
 *  Date Created: 05/10/2022
 *    
 *  ----- PRELIMINARY NOTES ------ 
 *  The following code is used to calibrate the servo motors to a neutral position of 135 degrees.   
 *  Note that the servo motors (SF3218MG) can rotate from 0 to 270 degrees so 135 degrees allows for 135 degrees of travel clockwise and counter-clockwise
 *  
 */

#include <Wire.h>                       // library for I2C communication
#include <Adafruit_PWMServoDriver.h>    // library for PCA9685 servo driver 

// create instance of Adafruit_PWMServoDriver class with default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// servo constants
#define SERVOMIN  85      // minimum servo pulse length count (out of 4096)
#define SERVOMAX  460     // maximum servo pulse length count (out of 4096)
#define USMIN  500        // rounded minimum microsecond length based on the minimum pulse of 80
#define USMAX  2500       // rounded maximum microsecond length based on the maximum pulse of 470
#define SERVO_FREQ 50     // analog servo frequency at 50Hz or pulse every 20ms
#define SERVO_COUNT 1     // calibrate one servo at a time

int calibration_angle[8] = {135, 135, 135, 135, 135, 135, 135, 135};    // servo position array
              
// function prototypes
void test_servo_sweep();
void calibrate_servos();
int map_angle(int angle);

// ----------------------------------------------------------------- SETUP ----------------------------------------------------------------

void setup() {
  Serial.begin(9600);                                 // establishing serial connection at baud rate of 9600 bits/s
  Serial.println("-8 channel Servo Calibration!-");
  
  pwm.begin();                                        // being pwm object
  pwm.setOscillatorFrequency(27000000);               // set IC oscillator frequency to get expected pmw update frequency 
  pwm.setPWMFreq(SERVO_FREQ);                         // set pwm frequency based on servo operating frequency

  test_servo_sweep();                                 // run sweep test
  calibrate_servos();                                 // run servo calibration
}

// ----------------------------------------------------------------- LOOP ----------------------------------------------------------------

void loop() {
  // EMPTY  
}

// --------------------------------------------------------------- FUNCTIONS --------------------------------------------------------------

// fuction that sweeps servos from 0 to 270 degrees and back
void test_servo_sweep() {
  for(int servo_num = 0 ; servo_num < SERVO_COUNT; servo_num++) {
      int pulse_len;
      
      
      // start from 0 degrees (extreme left end position)
      pulse_len = map_angle(0);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(8000);  // larger delay to out on servo horn to visualise movement

      pulse_len = map_angle(45);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      pulse_len = map_angle(90);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 135 degrees (neutral position)
      pulse_len = map_angle(135);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      pulse_len = map_angle(180);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      pulse_len = map_angle(225);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 270 degrees (extreme right position)
      pulse_len = map_angle(270);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);
  }
}


// function that sets servo to 135 angle position
void calibrate_servos() {
  for(int servo_num = 0 ; servo_num < SERVO_COUNT; servo_num++) {              // iterate through updated angle for each servo
    int pulse_len = map_angle(calibration_angle[servo_num]);                         // set pulse_length by mapping angle       
    pwm.setPWM(servo_num, 0, pulse_len);                                       // move servo to angle position
  }  
}


// function to map angle to pulse length
int map_angle(int angle) {
  // int microsecond_mapped_angle = map(angle, 0, 270, USMIN, USMAX);            // mapping with microseconds
  int pulse_length_angle = map(angle, 0, 270, SERVOMIN, SERVOMAX);               // mapping with min, max servo pulse length
  return pulse_length_angle;
}
