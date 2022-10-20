
// code to test ultrasonic sensor for commanding a robot dog 

// ultrasonic sensor pins
const int echo_pin = 2;
const int trig_pin = 3;
int distance_cm;
long duration_us;

// ultrasonic thresholds
const int LAYDOWN_THRES = 5;
const int STANDUP_THRES = 10;
const int SIT_THRES = 15;
const int SHAKE_THRES = 20;
const int DANCE_THRES = 25;



void setup() {
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  
  Serial.begin(9600);

}


void loop() {

  // use trigger pin to generate pulse
  digitalWrite(trig_pin, LOW);             // initially setting trig_pin to get a clean HIGH pulse
  delayMicroseconds(5);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);              
  digitalWrite(trig_pin, LOW);

  // measure duration of pulse from echo pin
  duration_us = pulseIn(echo_pin, HIGH);

  // calculate the distance
  distance_cm = 0.017 * duration_us;

  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm\n");

  if(distance_cm <= LAYDOWN_THRES) {
    Serial.println("Laydown command EXECUTED");
    delay(2000);
  }

  else if((distance_cm > LAYDOWN_THRES) && (distance_cm <= STANDUP_THRES)) {
    Serial.println("Standup command EXECUTED");
    delay(2000);
  }

  else if((distance_cm > STANDUP_THRES) && (distance_cm <= SIT_THRES)) {
    Serial.println("Sit command EXECUTED");
    delay(2000);
  }

  else if((distance_cm > SIT_THRES) && (distance_cm <= SHAKE_THRES)) {
    Serial.println("Shake command EXECUTED");
    delay(2000);
  }

  else if(distance_cm > SHAKE_THRES && (distance_cm <= DANCE_THRES)) {
    Serial.println("Dance command EXECUTED");
    delay(2000);
  }

  delay(1000);
 
}
