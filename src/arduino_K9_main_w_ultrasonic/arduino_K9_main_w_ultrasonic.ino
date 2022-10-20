
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
 *  Using ultrasonic sensor to control motion
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
#define SERVO_COUNT 8         // number of servo actuators (8 leg servos + 1 head servo)

// ultrasonic initialisations
const int echo_pin = 2;
const int trigger_pin = 3;
int distance_cm;
long duration_us;
const int LAYDOWN_THRES = 5;
const int STANDUP_THRES = 10;
const int SIT_THRES = 15;
const int SHAKE_THRES = 20;
const int DANCE_THRES = 25;

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
int updated_servo_angle[8];              // updated servo positions

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

  // pins for ultrasonic sensor
  pinMode(trigger_pin, OUTPUT);             // Sets the trig_pin as an OUTPUT
  pinMode(echo_pin, INPUT);                 // Sets the echo_pin as an INPUT
  
  Serial.begin(9600);                       // establishing serial connection at baud rate of 9600 bits/s
  
  pwm.begin();                              // being pwm object
  pwm.setOscillatorFrequency(27000000);     // set IC oscillator frequency to get expected pmw update frequency 
  pwm.setPWMFreq(SERVO_FREQ);               // set pwm frequency based on servo operating frequency

}

// ------------------------------------------------------------------- LOOP -------------------------------------------------------------------

void loop() {

  // get distance from ultrasonic sensor
  digitalWrite(trig_pin, LOW);                  // initially setting trig_pin LOW to get a clean HIGH pulse
  delayMicroseconds(5);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);                        // use trigger pin to generate pulse 
  digitalWrite(trig_pin, LOW);

 
  duration_us = pulseIn(echo_pin, HIGH);        // measure duration of pulse from echo pin

  distance_cm = 0.017 * duration_us;            // calculate distance to bounding object

  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm\n");

  // command K9 based on distance to bounding object (hand)
  if(distance_cm <= LAYDOWN_THRES) {
    laydown();
    Serial.println("Laydown command EXECUTED");
    delay(1000);
  }

  else if((distance_cm > LAYDOWN_THRES) && (distance_cm <= STANDUP_THRES)) {
    standup();
    Serial.println("Standup command EXECUTED");
    delay(1000);
  }

  else if((distance_cm > STANDUP_THRES) && (distance_cm <= SIT_THRES)) {
    sit();
    Serial.println("Sit command EXECUTED");
    delay(1000);
  }

  else if((distance_cm > SIT_THRES) && (distance_cm <= SHAKE_THRES)) {
    shake();
    Serial.println("Shake command EXECUTED");
    delay(1000);
  }

  else if(distance_cm > SHAKE_THRES && (distance_cm <= DANCE_THRES)) {
    dance();
    Serial.println("Dance command EXECUTED");
    delay(1000);
  }

  delay(1000);
  
}

// ------------------------------------------------------------------- MOTOR CONTROL -------------------------------------------------------------------

// function that moves servos to correct positions 
void set_servo_position() {

  int pulse_len;
  
  // iterate through updated angle for each servo
  for(int servo_num = 0; servo_num < SERVO_COUNT; servo_num++) {          
    
    // controls movement of leg servos
    pulse_len = map_angle(updated_servo_angle[servo_num], 270);           // map updated servo angle position to assocaited pulse length value
    pwm.setPWM(servo_num, 0, pulse_len);                                  // move servo to specified position 
  }  
}

// function to map angle to pulse length 
int map_angle(int angle, int max_deg) {
  int pulse_length_angle = map(angle, 0, 270, SERVOMIN, SERVOMAX);        // mapping with min, max servo pulse length
  return pulse_length_angle; 
 
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
  }
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------ 
