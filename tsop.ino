#include <IRremote.h>

int RECV_PIN = 11;

int result;
IRrecv irrecv(RECV_PIN);

decode_results results;

void setup()
{ pinMode(11, INPUT);
pinMode(2,OUTPUT);
pinMode(3,OUTPUT);
pinMode(4,OUTPUT);
pinMode(5,OUTPUT);
pinMode(6,OUTPUT);
pinMode(7,OUTPUT);
pinMode(8,OUTPUT);
pinMode(9,OUTPUT);
  Serial.begin(9600);
        digitalWrite(2, LOW);  
        digitalWrite(3, LOW);
        digitalWrite(4, LOW);
        digitalWrite(5, LOW);
      irrecv.enableIRIn(); // Start the receiver
}
void loop() {
  result=results.value;
  if (irrecv.decode(&results)) {
    Serial.println(results.value, DEC);
   if(result == 1)//front
     {   Serial.print("pin2 LOw\n");
            digitalWrite(2, LOW);
                                
       
       irrecv.resume(); // Receive the next value  
   }
     else if(result == 2049)
     {Serial.print("pin 2 high\n");
          digitalWrite(2, HIGH);
             
       irrecv.resume(); // Receive the next value  
   }
    else if( result == 2)//right
      {Serial.print("pin 3 LOW\n");
        digitalWrite(3, LOW);
                
        irrecv.resume(); // Receive the next value  
    }
    else if( result == 2050)
      {
       digitalWrite(3,HIGH);
              
    irrecv.resume(); // Receive the next value
      }
    else if( result == 3)//back
      {
        digitalWrite(4,LOW);
                      Serial.print("pin 4 low\n");
           
        irrecv.resume(); // Receive the next value  
    }
    else if( result == 2051)
      {
        digitalWrite(4,HIGH);
                     Serial.print("pin 4 high\n");
        
    irrecv.resume(); // Receive the next value
      }
    else if( result == 4)
      {
        digitalWrite(5,LOW);
                     Serial.print("pin 5 low\n");
                          irrecv.resume(); // Receive the next value
      }
    else if( result == 2052)
      {
        digitalWrite(5,HIGH);
                     Serial.print("pin 5 HIGH\n");
    irrecv.resume(); // Receive the next value                     
      }
    irrecv.resume(); // Receive the next value                     
   
  }
}
