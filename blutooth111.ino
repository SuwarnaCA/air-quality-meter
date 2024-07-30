
#include<SoftwareSerial.h>
int state = 9;
#define rx 0
#define tx 1
SoftwareSerial bt(rx, tx);
void setup() {
pinMode(4, OUTPUT);
pinMode(8, OUTPUT);
pinMode(11, OUTPUT);
              digitalWrite(4,0);
              digitalWrite(8,0);
               Serial.begin(9600); 
               }

void loop() {

          state = Serial.read();
         
      Serial.println(state);
      Serial.print(" ");

        switch(state)
        {
      case 82:digitalWrite(4,LOW);
              digitalWrite(5,LOW);
              digitalWrite(8,HIGH);
              digitalWrite(9,LOW);
              Serial.println("right");
                break;
      case 76:digitalWrite(4,HIGH);
              digitalWrite(5,LOW);
              digitalWrite(8,LOW);
              digitalWrite(9,LOW);
                 Serial.println("left");
                break;
      case 70:digitalWrite(8,LOW);
              digitalWrite(9,HIGH);
              digitalWrite(4,LOW);
              digitalWrite(5,HIGH);
              Serial.println("forward");
                break;
      case 66:digitalWrite(4,HIGH);
              digitalWrite(5,LOW);
              digitalWrite(9,LOW);
              digitalWrite(8,HIGH);
              Serial.println("Back");
                break;
      case 83:
              digitalWrite(4,LOW);
              digitalWrite(5,LOW);
              digitalWrite(9,LOW);
              digitalWrite(8,LOW);
                   Serial.println("stop");
                break;
     }
     delay(100);
}
