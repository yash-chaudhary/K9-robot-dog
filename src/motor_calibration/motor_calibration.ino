
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
#define SERVOMIN  85          // minimum servo pulse length count (out of 4096)
#define SERVOMAX  460         // maximum servo pulse length count (out of 4096)
#define USMIN  500            // rounded minimum microsecond length based on the minimum pulse of 80
#define USMAX  2500           // rounded maximum microsecond length based on the maximum pulse of 470
#define SERVO_FREQ 50         // analog servo frequency at 50Hz or pulse every 20ms
#define SERVO_COUNT 1         // calibrate one servo at a time
#define SG90_SERVOMIN 70      // minimum servo pulse length count for SG-90 head motor
#define SG90_SERVOMAX 490     // maximum servo pulse length count for SG-90 head motor

/* 
 *  NOTE that only one set of servos are calibrated at a time to ensure proper calibration
 *
 * first angles is used to calibrate SF3218MG servo motors (max angle of 270 deg)
 * last angle used to calibrate SG-90 servo motor (max angle of 180 deg)
 */ 
int calibration_angle[3] = {135, 90};    // servo position array
char servo_command;                      // servo command character in serial monitor
              
// function prototypes
void test_servo_sweep(int max_deg);
void calibrate_servos(int max_deg);
int map_angle(int angle, int max_deg);


// ----------------------------------------------------------------- SETUP ----------------------------------------------------------------

void setup() {
  Serial.begin(9600);                                                         // establishing serial connection at baud rate of 9600 bits/s
  Serial.println("---- INITIATING Servo Motor Calibration and Sweep Test ---- ");
  Serial.println();
  
  pwm.begin();                                                                // being pwm object
  pwm.setOscillatorFrequency(27000000);                                       // set IC oscillator frequency to get expected pmw update frequency 
  pwm.setPWMFreq(SERVO_FREQ);                                                 // set pwm frequency based on servo operating frequency  
}

// ----------------------------------------------------------------- LOOP ----------------------------------------------------------------

// loop to run servo commands
void loop() {
  
  while(Serial.available()) {
    servo_command = Serial.read();
      switch(servo_command) {
        
        case 'q' :
          Serial.println("Initiating SG-90 sweep test...");
          test_servo_sweep(180);
          Serial.println(" ---- SG-90 sweep test complete ----");
          Serial.println();
          break;

        case 'w': 
          Serial.println("Initiating SG-90 calibration...");
          calibrate_servos(180);
          Serial.println(" ---- SG-90 calibration COMPLETE ----");
          Serial.println();
          break;

        case 'e':
          Serial.println("Initiating SF3218MG sweep test...");
          test_servo_sweep(270);
          Serial.println(" ---- SF3218MG sweep test COMPLETE ----");
          Serial.println();
          break;

        case 'r':
          Serial.println("Initiating SF3218MG calibration...");
          calibrate_servos(270);
          Serial.println(" ---- SF3218MG calibration COMPLETE ----");
          Serial.println();
          break;
      }
  }
}

// --------------------------------------------------------------- FUNCTIONS --------------------------------------------------------------

// fuction that sweeps leg servos from 0 to 270 degrees and back
void test_servo_sweep(int max_deg) {
  
  for(int servo_num = 0 ; servo_num < SERVO_COUNT; servo_num++) {
    int pulse_len;

    if (max_deg == 270) {
      
      // start from 0 degrees (extreme left end position)
      pulse_len = map_angle(0, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(8000);  // larger delay to out on servo horn to visualise movement

      // move to 45 deg
      pulse_len = map_angle(45, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 90 deg
      pulse_len = map_angle(90, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 135 degrees (neutral position)
      pulse_len = map_angle(135, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 180 deg
      pulse_len = map_angle(180, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 225 deg
      pulse_len = map_angle(225, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 270 degrees (extreme right position)
      pulse_len = map_angle(270, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);
    }

    else if (max_deg == 180) {

      // start motion at 0 deg
      pulse_len = map_angle(0, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(8000);                           // larger delay to out on servo horn to visualise 

      // move to 45 deg
      pulse_len = map_angle(45, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 90 deg
      pulse_len = map_angle(90, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 135 deg
      pulse_len = map_angle(135, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);

      // move to 180 deg
      pulse_len = map_angle(180, max_deg);
      pwm.setPWM(servo_num, 0, pulse_len);
      delay(5000);
    }
  }
}


// function that sets servo to 135 angle position
void calibrate_servos( int max_deg) {
  if (max_deg == 180) {
    int pulse_len =  map_angle(calibration_angle[1], max_deg);        // set pulse_length by mapping 90 deg angle 
    pwm.setPWM(0, 0, pulse_len);                                      // move servo on Pin 0 to angle position
  }
  
  else if (max_deg == 270) {
    int pulse_len =  map_angle(calibration_angle[0], max_deg);        // set pulse_length by mapping 135 deg angle 
    pwm.setPWM(0, 0, pulse_len);                                      // move servo on Pin 0 to angle position
  }
}


// function to map angle to pulse length 
int map_angle(int angle, int max_deg) {

  // maps pulse length between 0 and 180
  if (max_deg == 180) {
    int pulse_length_angle = map(angle, 0, 180, SG90_SERVOMIN, SG90_SERVOMAX);     // mapping with min, max servo pulse length
    return pulse_length_angle; 
  }

  // maps pulse length between 0 and 270
  else if (max_deg == 270) {
    // int microsecond_mapped_angle = map(angle, 0, 270, USMIN, USMAX);            // mapping with microseconds
    int pulse_length_angle = map(angle, 0, 270, SERVOMIN, SERVOMAX);               // mapping with min, max servo pulse length
    return pulse_length_angle; 
  }
}
