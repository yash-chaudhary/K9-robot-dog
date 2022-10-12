void setup() {
  Serial.begin(9600); // set baud rate to 9600 bits/s
  
}

void loop() {
   while(Serial.available()) {
      String data = Serial.readStringUntil('/n');       // Arduino reads data sent from RPI
      Serial.print("(ARDUINO SAYS) You sent me: ");     // Arduino sends its own message
      Serial.println(data);                             // Arduino send message it received from RPI
   }
}
