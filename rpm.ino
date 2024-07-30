
volatile byte tick;
unsigned int rpm;
unsigned long timeold;
volatile byte tick1;
unsigned int rpm1;
unsigned long timeold1;

void setup(){
  Serial.begin(9600);                      //Initialize Serial port communication;
  attachInterrupt(0, tick_count, RISING);
  attachInterrupt(1, tick1_count, RISING);  //Execute interrupt function (tick_count) defined below each rising edge on external interrupt 0 (pin 2);
  pinMode(18, OUTPUT);                      //Set PWM pin 18 to output to motor driver;
  pinMode(2, INPUT);  //Set interrupt port 2 to input from proximity sensor;
  pinMode(3,INPUT);
  pinMode(A0, INPUT);                      //Set ADC pin A0 to input from potentiometer;
}  

//Interrupt function;
void tick_count()                          //Executed every rising edge on interrupt 0 (pin 2)
{  
    tick++;     //Increment "tick" by 1;
}
void tick1_count()
{  
    tick1++;     //Increment "tick" by 1;
}
void loop() {
   int command1;
  int pwm1;
  int duty_cycle1;
  
  command1 = analogRead(1);                 //Read analog signal on channel 0 of ADC and convert it into a 10-bit digital integer and store it in variable "command";
  pwm1 = map(command1,0,1023,0,255);         //Scale the 10-bit digital value of the ADC with the 8-bit resolution of the PWM output to motor driver;  
  duty_cycle1 = pwm1*100/255;                //Calculate PWM duty cycle in %;
  analogWrite(5,pwm1);                      //Send the previously specified pwm signal on pin 5 to motor driver;
  
  //Start RPM calculations;
  if(tick1>=1)                                       //When a full fan cycle is completed (fan has 4 blades)
   {
    rpm1 = 15*1000/(millis() - timeold1)*tick1;        //Divide tick counter "tick" by the time it took to do the 4 ticks using the "millis()" function. 60/4=15 giving tyhe actual RPM of a 4-blade fan;
    timeold1 = millis();                             //Save the previous time to use it for next iteration of speed calculation;
    tick1 = 0;                                       //Reset "tick" to zero to prepare it for the next iteration of speed calculation;
   }
  int command;
  int pwm;
  int duty_cycle;
  
  command = analogRead(0);                 //Read analog signal on channel 0 of ADC and convert it into a 10-bit digital integer and store it in variable "command";
  pwm = map(command,0,1023,0,255);         //Scale the 10-bit digital value of the ADC with the 8-bit resolution of the PWM output to motor driver;  
  duty_cycle = pwm*100/255;                //Calculate PWM duty cycle in %;
  analogWrite(5,pwm);                      //Send the previously specified pwm signal on pin 5 to motor driver;
  
  //Start RPM calculations;
  if(tick>=1)                                       //When a full fan cycle is completed (fan has 4 blades)
   {
    rpm = 15*1000/(millis() - timeold)*tick;        //Divide tick counter "tick" by the time it took to do the 4 ticks using the "millis()" function. 60/4=15 giving tyhe actual RPM of a 4-blade fan;
    timeold = millis();                             //Save the previous time to use it for next iteration of speed calculation;
    tick = 0;                                       //Reset "tick" to zero to prepare it for the next iteration of speed calculation;
   }
  //End RPM Calculations
  
  //Send desired data over the Serial COM Port;
  Serial.print("Command = ");    
  Serial.print(command);
   
  Serial.print("  ");
  Serial.print("PWM = "); 
  Serial.print(duty_cycle);
   
  Serial.print("   RPM = ");
  Serial.println(rpm);
   
   Serial.print("Command1 = ");    
  Serial.print(command1);
   Serial.print("  ");
  Serial.print("PWM1 = ");
  Serial.print(duty_cycle1);
   Serial.print("   RPM1 = ");
  Serial.println(rpm1);
  delay(150);
}
 

