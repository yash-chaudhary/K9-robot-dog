
String command;                 // string type command

void standup();
void laydown();
void sit();
void shake();
void dance();
void walk();


// ------------------------------------------------------------------- SETUP -------------------------------------------------------------------

void setup() {
  Serial.begin(115200);                               // establishing serial connection at baud rate of 9600 bits/s
  
  pinMode(LED_BUILTIN, OUTPUT);                     // initialize digital pin LED_BUILTIN as an output.

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

    if (command.equals("1")) {
      laydown();                              // execute laydown routine
      Serial.println(1);                      // send execution success integer 
    }
    
    else if (command.equals("2")) {
      standup();                              // execute standup routine
      Serial.println(1);                      
    }

    else if (command.equals("3")) {
        sit();                                // execute sit routine
        Serial.println(1);                    
    }

    else if (command.equals("4")) {
        shake();                              // execute sit routine
        Serial.println(1);                    
    }

    else if (command.equals("5")) {
        dance();                              // execute dance routine
        Serial.println(1);                     
    } 

    // runtime protection just in case some bad data is transferred
    else {
        standup();                            // execute standup routine
        Serial.println(1);                     
    }

    delay(1000);  
  }
}


// ---------------------------------------------------------------------- ROUTINES ---------------------------------------------------------------------- 

// laydown routune (assigned to 1 finger up so blink once)
void laydown() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);   
}


// standup routine (assigned to 2 fingers up so blink twice)
void standup() { 
  for(int i=0; i < 2; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);     
  }
}



// sit routine (assigned to 3 fingers up so blink 3 times)
void sit() {
  
  for(int i=0; i < 3; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);     
  }
}


// shake routine (assigned to four fingers up so blink 4 times)
void shake() {

  for(int i=0; i < 4; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000); 
  }
}

// dance routine (assigned to 5 fingers up so blink 5 times)
void dance() {

  for(int i=0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                       // wait for a second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000); 
  } 
}


// ------------------------------------------------------------------------------------------------------------------------------------------------------ 
