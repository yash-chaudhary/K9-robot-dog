
/*  
 *  Project Name: K9
 *  Author: Yash Chaudhary
 *  Date Created: 04/10/2022
 *    
 *    ------ PRELIMINARY NOTES ------ 
 *  Servo motors SF3218MG can move from 0 to 270 degrees and are mounted on chassis sideways.
 *  Before mounting upper and lower leg assembly run the motor calibration code that will set the motors to the neutral position at 90 degrees.
 *  The servo neutral position can be moved to 135 degrees giving the robot +/- 135 degrees of motion at the joints. 
 *  The kinematic model chosen for this project is forward kinematics where the angles of the joints are specified to move the robot to its desired configuration
 *  This is a brute force method used to actuate the robot by essentially finding the angles of movement via trial and error
 *  
 */

#include <Wire.h>                       // library for I2C communication
#include <math.h>                       // library for math operations
#include <Adafruit_PWMServoDriver.h>    // library for PCA9685 servo driver 

// create instance of Adafruit_PWMServoDriver class with default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// servo constants
#define SERVOMIN  80      // minimum servo pulse length count (out of 4096)
#define SERVOMAX  470     // maximum servo pulse length count (out of 4096)
#define USMIN  500        // rounded minimum microsecond length based on the minimum pulse of 80
#define USMAX  2500       // rounded maximum microsecond length based on the maximum pulse of 470
#define SERVO_FREQ 50     // analog servo frequency at 50Hz or pulse every 20ms
#define SERVO_COUNT 8     // number of servo actuators


char command;                                                 // char type command
int init_servo_angle[8] = {90, 90, 90, 90, 90, 90, 90, 90};   // initial servo positions
int updated_servo_angle[8];                                   // updated servo positions

// function prototypes
void set_servo_position();
void standup();
void laydown();
void sit();
void shake();
void dance();
void walk();


// ------------------------------------------------------------------- SETUP -------------------------------------------------------------------


void setup() {
  Serial.begin(9600);                       // establishing serial connection at baud rate of 9600 bits/s
  Serial.println("8 channel Servo test!");
  
  pwm.begin();                              // being pwm object
  pwm.setOscillatorFrequency(27000000);     // set IC oscillator frequency to get expected pmw update frequency 
  pwm.setPWMFreq(SERVO_FREQ);               // set pwm frequency based on servo operating frequency
}


// ------------------------------------------------------------------- LOOP -------------------------------------------------------------------


/*
 *  --- command control ---
 * 'q' = standup()
 * 'w' = laydown()
 * 'e' = sit()
 * 'r' = shake()
 * 't' = dance()
 * 'y' = walk()
*/

void loop() {
  while(Serial.available()) {
    command = Serial.read();
      switch(command) {

      case 'q' :
        Serial.println("----- Standup command executed! -----");
        standup();
      break;

      case 'w' :
        Serial.println("----- Laydown command executed! -----");
        laydown();
      break;

      case 'e' :
        Serial.println("----- Sit command executed! -----");
        sit();
      break;

      case 'r' :
        Serial.println("----- Shake command executed! -----");
        shake();
      break;

      case 't' :
        Serial.println("----- Dance command executed! -----");
        dance();
      break;

      case 'y' :
        Serial.println("----- Walk command executed! -----");
        walk();
      break;
      }
  }
}


// ------------------------------------------------------------------- MOTOR CONTROL -------------------------------------------------------------------

// NEED to setup up a way to revert back to standup movement after a delay for each movement !!!!!!!!!!!!!!

// function that moves servos to correct positions 
void set_servo_position() {
  int i;
  for(int servo_num; servo_num < SERVO_COUNT; servo_num++) {                           // iterate through updated angle for each servo
    int microsecond_mapped_angle = map(updated_servo_angle[i], 0, 270, 500, 2500);     // map updated servo angle position to assocaited microsecond value
    pwm.writeMicroseconds(i, microsecond_mapped_angle);                                // move servo to specified position
  }  
}


// ---------------------------------------------------------------------- ROUTINES ---------------------------------------------------------------------- 

// standup routine
void standup() {
  
//  updated_servo_angle[0] = // front left upper leg 
//  updated_servo_angle[1] = // front left lower leg
//  updated_servo_angle[2] = // back left upper leg
//  updated_servo_angle[3] = // back left lower leg
//  updated_servo_angle[4] = // front right upper leg
//  updated_servo_angle[5] = // front right lower leg
//  updated_servo_angle[6] = // back right upper leg
//  updated_servo_angle[7] = // back right lower leg

  set_servo_position();
}


// laydown routune
void laydown() {
  updated_servo_angle[0] = 90;   // front left upper leg 
  updated_servo_angle[1] = 90;   // front left lower leg
  updated_servo_angle[2] = 90;   // back left upper leg
  updated_servo_angle[3] = 90;   // back left lower leg
  updated_servo_angle[4] = 90;   // front right upper leg
  updated_servo_angle[5] = 90;   // front right lower leg
  updated_servo_angle[6] = 90;   // back right upper leg
  updated_servo_angle[7] = 90;   // back right lower leg

  set_servo_position();
}


// sit routine
void sit() {

//  updated_servo_angle[0] = // front left upper leg 
//  updated_servo_angle[1] = // front left lower leg
//  updated_servo_angle[2] = // back left upper leg
//  updated_servo_angle[3] = // back left lower leg
//  updated_servo_angle[4] = // front right upper leg
//  updated_servo_angle[5] = // front right lower leg
//  updated_servo_angle[6] = // back right upper leg
//  updated_servo_angle[7] = // back right lower leg
  
}


// shake routine
void shake() {

//  updated_servo_angle[0] = // front left upper leg 
//  updated_servo_angle[1] = // front left lower leg
//  updated_servo_angle[2] = // back left upper leg
//  updated_servo_angle[3] = // back left lower leg
//  updated_servo_angle[4] = // front right upper leg
//  updated_servo_angle[5] = // front right lower leg
//  updated_servo_angle[6] = // back right upper leg
//  updated_servo_angle[7] = // back right lower leg
  
}


// dance routine
void dance() {
  
}


// walk routine
void walk() {
  
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------ 
