//Woon Jun Shen
//UM402 (433 MHz UART)
//#include <SoftwareSerial.h>

//SoftwareSerial mySerial(2, 3); //TX, RX
// gnd SET_A and SET_B for Normal Mode (Send and Receive)

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
}

void loop() {
  
  
  if(Serial.available() > 0){//Read from serial monitor and send over UM402
    String input = Serial.readString();
    Serial1.println(input);    
  }
 
  if(Serial1.available() > 1){//Read from UM402 and send to serial monitor
    String input = Serial1.readString();
    Serial.println(input);    
  }
  delay(20);
}
