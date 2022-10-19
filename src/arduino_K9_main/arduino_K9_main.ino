
/*  
 *  Project Name: K9
 *  Author: Yash Chaudhary
 *  Date Created: 04/10/2022
 *    
 *    ------ PRELIMINARY NOTES ------ 
 *  Servo motors SF3218MG can move from 0 to 270 degrees and are mounted on chassis sideways.
 *  Before mounting upper and lower leg assembly run the motor calibration code that will set the motors to the neutral position at 135 degrees.
 *  The servo neutral position can be moved to 135 degrees giving the robot +/- 135 degrees of motion at the joints. 
 *  The kinematic model chosen for this project is forward kinematics where the angles of the joints are specified to move the robot to its desired configuration
 *  This is a brute force method used to actuate the robot by essentially finding the angles of movement via trial and error
 *  
 *  NOTE that the head will move only for certain commands and operates betweeo 0 and 180 degrees where 90 degrees is its neutral position
 *  
 */

#include <Wire.h>                       // library for I2C communication
#include <Adafruit_PWMServoDriver.h>    // library for PCA9685 servo driver 

// create instance of Adafruit_PWMServoDriver class with default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40, Wire);

// servo constants
#define SERVOMIN  85          // minimum servo pulse length count (out of 4096)
#define SERVOMAX  460         // maximum servo pulse length count (out of 4096)
#define SG90_SERVOMIN 70      // minimum servo pulse length count for SG-90 head motor
#define SG90_SERVOMAX 490     // maximum servo pulse length count for SG-90 head motor
#define USMIN  500            // rounded minimum microsecond length based on the minimum pulse of 80
#define USMAX  2500           // rounded maximum microsecond length based on the maximum pulse of 470
#define SERVO_FREQ 50         // analog servo frequency at 50Hz or pulse every 20ms
#define SERVO_COUNT 9         // number of servo actuators (8 leg servos + 1 head servo)

String command;                 // string type command

/*
 * updated_servo_angle[0] assigned to front right upper leg
 * updated_servo_angle[1] assigned to front right lower leg
 * updated_servo_angle[2] assigned to front left upper leg
 * updated_servo_angle[3] assigned to front left lower leg
 * updated_servo_angle[4] assigned to back right upper leg
 * updated_servo_angle[5] assigned to back right lower leg
 * updated_servo_angle[6] assigned to back left upper leg
 * updated_servo_angle[7] assigned to back left upper leg
 * updated_servo_angle[8] assigned to head 
 */
const int init_leg_servo_angle = 135;    // initial leg servo positions
const int init_head_servo_angle = 90;    // intial head servo position
int updated_servo_angle[9];              // updated servo positions

// function prototypes
void set_servo_position();
int map_angle(int angle, int max_deg);
void standup();
void laydown();
void sit();
void shake();
void dance();
void walk();


// ------------------------------------------------------------------- SETUP -------------------------------------------------------------------

void setup() {
  Serial.begin(9600);                               // establishing serial connection at baud rate of 9600 bits/s
  
  pwm.begin();                                      // being pwm object
  pwm.setOscillatorFrequency(27000000);             // set IC oscillator frequency to get expected pmw update frequency 
  pwm.setPWMFreq(SERVO_FREQ);                       // set pwm frequency based on servo operating frequency

  // ensure serial connection established before proceeding
  while(!Serial) {
    ;
  }
}

// ------------------------------------------------------------------- LOOP -------------------------------------------------------------------

void loop() {

  // serial connection established between RPI and Arduino
  if(Serial.available()) {

    command = Serial.readStringUntil('\n');       // read string from RPI
    command.trim();                               // remove any whitespace

    if (command.equals("laydown")) {
      laydown();                              // execute laydown routine
      Serial.println(1);                      // send execution success integer 
    }

    else if (command.equals("standup")) {
      standup();                              // execute standup routine
      Serial.println(1);                      
    }

    else if (command.equals("sit")) {
        sit();                                // execute sit routine
        Serial.println(1);                    
    }

    else if (command.equals("shake")) {
        shake();                              // execute sit routine
        Serial.println(1);                    
    }

    else if (command.equals("dance")) {
        dance();                              // execute dance routine
        Serial.println(1);                     
    } 

    // runtime protection just in case some bad data is transferred
    else {
        standup();                              // execute standup routine
        Serial.println(1);                     
    }

    delay(1000);  
  }
}

// ------------------------------------------------------------------- MOTOR CONTROL -------------------------------------------------------------------

// NEED to setup up a way to revert back to standup movement after a delay for each movement !!!!!!!!!!!!!! - just use a delay and then standup() command for non-standups commands

// function that moves servos to correct positions 
void set_servo_position() {

  int pulse_len;
  
  // iterate through updated angle for each servo
  for(int servo_num = 0; servo_num < SERVO_COUNT; servo_num++) {          

    // controls movement of head servo
    if (servo_num == 8) {
      pulse_len = map_angle(updated_servo_angle[servo_num], 180);           // map updated servo angle position to assocaited pulse length value
      pwm.setPWM(servo_num, 0, pulse_len);                                  // move servo to specified position
    }

    // controls movement of leg servos
    else {
      pulse_len = map_angle(updated_servo_angle[servo_num], 270);           // map updated servo angle position to assocaited pulse length value
      pwm.setPWM(servo_num, 0, pulse_len);                                  // move servo to specified position
    }
    
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
    int pulse_length_angle = map(angle, 0, 270, SERVOMIN, SERVOMAX);               // mapping with min, max servo pulse length
    return pulse_length_angle; 
  }
}


// ---------------------------------------------------------------------- ROUTINES ---------------------------------------------------------------------- 

// standup routine 
void standup() {

  // can add a delay to maybe standup back legs before front legs 
  updated_servo_angle[0] = init_leg_servo_angle - 55;       // front right upper leg 
  updated_servo_angle[1] = init_leg_servo_angle - 25;       // front right lower leg
  
  updated_servo_angle[2] = init_leg_servo_angle + 40;       // front left upper leg
  updated_servo_angle[3] = init_leg_servo_angle + 30;       // front left lower leg
  
  updated_servo_angle[4] = init_leg_servo_angle - 55;       // back right upper leg
  updated_servo_angle[5] = init_leg_servo_angle - 25;       // back right lower leg
  
  updated_servo_angle[6] = init_leg_servo_angle + 40;       // back left upper leg
  updated_servo_angle[7] = init_leg_servo_angle + 30;       // back left lower leg

  set_servo_position();
}


// laydown routune 
void laydown() {
  
   updated_servo_angle[0] = init_leg_servo_angle;      // front right upper leg (FRU)
   updated_servo_angle[1] = init_leg_servo_angle;      // front right lower leg (FRL)
   
   updated_servo_angle[2] = init_leg_servo_angle;      // front left upper leg  (FLU)
   updated_servo_angle[3] = init_leg_servo_angle;      // front left lower leg   (FLL)
   
   updated_servo_angle[4] = init_leg_servo_angle;      // back right upper leg (BRU)
   updated_servo_angle[5] = init_leg_servo_angle;      // back right lower leg (BRL)
   
   updated_servo_angle[6] = init_leg_servo_angle;      // back left upper leg (BLU)
   updated_servo_angle[7] = init_leg_servo_angle;      // back left lower leg (BLL)

   set_servo_position();
}


// sit routine 
void sit() {

   updated_servo_angle[0] = init_leg_servo_angle - 45;       // front right upper leg 
   updated_servo_angle[1] = init_leg_servo_angle - 45;       // front right lower leg
  
   updated_servo_angle[2] = init_leg_servo_angle + 45;       // front left upper leg
   updated_servo_angle[3] = init_leg_servo_angle + 45;       // front left lower leg
   
   updated_servo_angle[4] = init_leg_servo_angle - 10;       // back right upper leg
   updated_servo_angle[5] = init_leg_servo_angle - 10;       // back right lower leg 
   
   updated_servo_angle[6] = init_leg_servo_angle + 10;       // back left upper leg
   updated_servo_angle[7] = init_leg_servo_angle + 10;       // back left lower leg 

   set_servo_position(); 
  
}


// shake routine
void shake() {

   updated_servo_angle[0] = init_leg_servo_angle + 20;       // front right upper leg 
   updated_servo_angle[1] = init_leg_servo_angle;            // front right lower leg
  
   updated_servo_angle[2] = init_leg_servo_angle + 45;       // front left upper leg
   updated_servo_angle[3] = init_leg_servo_angle + 45;       // front left lower leg
   
   updated_servo_angle[4] = init_leg_servo_angle - 10;       // back right upper leg
   updated_servo_angle[5] = init_leg_servo_angle - 10;       // back right lower leg 
   
   updated_servo_angle[6] = init_leg_servo_angle + 10;       // back left upper leg 
   updated_servo_angle[7] = init_leg_servo_angle + 10;       // back left lower leg

   set_servo_position(); 
}

// dance routine
void dance() {

  standup();          // must start in standup stance

  delay(1250);        // delay to allow servos to move to standup stance

  int moves_to_bust = 4;   // number of moves to be made

  // loop to make dance move
  for(int i = 0; i < moves_to_bust; i++) {
  
  // servo positions for moving up
  updated_servo_angle[0] = init_leg_servo_angle - 20;       // front right upper leg 
  updated_servo_angle[1] = init_leg_servo_angle - 40;       // front right lower leg
  
  updated_servo_angle[2] = init_leg_servo_angle + 15;       // front left upper leg
  updated_servo_angle[3] = init_leg_servo_angle + 45;       // front left lower leg

  updated_servo_angle[4] = init_leg_servo_angle - 20;       // back right upper leg
  updated_servo_angle[5] = init_leg_servo_angle - 50;       // back right lower leg
  
  updated_servo_angle[6] = init_leg_servo_angle + 15;       // back left upper leg
  updated_servo_angle[7] = init_leg_servo_angle + 45;       // back left lower leg

  set_servo_position();

  delay(1000);  // delay before changing back to standup stance

  // go back down to standing stance
  standup();

  delay(1000);
    
  }
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------ 
