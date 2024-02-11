//Woon Jun Shen
//UM402 (433 MHz UART)
#include <SoftwareSerial.h>
int input=0;
SoftwareSerial mySerial(2, 3); //TX, RX
// gnd SET_A and SET_B for Normal Mode (Send and Receive)

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
    input +=1;
    mySerial.println(input);    
    delay(500);
  if(mySerial.available() > 1){//Read from UM402 and send to serial monitor
    String input = mySerial.readString();
    Serial.println(input);    
  }
  delay(20);
}
