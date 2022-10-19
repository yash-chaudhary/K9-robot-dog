
/*
 * Code that controls the operation of Arduino built-in LED via a command from a Raspberry Pi via serial communication
 * 
*/

String command;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);   // initialize digital pin LED_BUILTIN as an output.

  // ensures serial connection is established before moving to loop
  while(!Serial) {
    ;
  }

}


// the loop function runs over and over again forever
void loop() {

  // checking is serial communication to RPI is available
  if(Serial.available()) {
    command = Serial.readStringUntil('\n');    // read string command
    command.trim();                            // removes any whitespace 
    
    if(command.equals("on")) {
      digitalWrite(LED_BUILTIN, HIGH);        // turn the LED on (HIGH is the voltage level)  
      Serial.println(1);                      // send int back to RPI to confirm command executed before next command issued
    }

    else if (command.equals("off")) {
      digitalWrite(LED_BUILTIN, LOW);          // turn the LED off (LOW is the voltage level) 
      Serial.println(1);                      // send int back to RPI to confirm command executed before next command issued        
    }
  }        
}
