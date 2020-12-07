#define THROTTLE_SIGNAL_IN 1 // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
#define THROTTLE_SIGNAL_IN_PIN 3 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead
#define LR_SIGNAL_IN 0 // INTERRUPT 0 = DIGITAL PIN 2 - use the interrupt number in attachInterrupt
#define LR_SIGNAL_IN_PIN 2 // INTERRUPT 0 = DIGITAL PIN 2 - use the PIN number in digitalRead

#define NEUTRAL_THROTTLE 1500 // this is the duration in microseconds of neutral throttle on an electric RC Car
#define NEUTRAL_LR 1500

#define PWM_VALUE 0      // from 0 to 255

#define PWM1 8
#define PWM2 9
#define PWM3 6
#define PWM4 7

#define ENB1 31
#define ENB2 33
#define ENB3 35
#define ENB4 37

#define DIR1 39
#define DIR2 41
#define DIR3 43
#define DIR4 45

volatile int nThrottleIn = NEUTRAL_THROTTLE; // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriod = 0; // set in the interrupt
volatile boolean bNewThrottleSignal = false; // set in the interrupt and read in the loop
volatile int nLRIn = NEUTRAL_LR; // volatile, we set this in the Interrupt and read it in loop so it must be declared volatile
volatile unsigned long ulStartPeriodLR = 0; // set in the interrupt
volatile boolean bNewLRSignal = false; // set in the interrupt and read in the loop

class motor{
  public:
    int pwm_pin;
    int dir;
    int enb;
    int dirSide;
    int offset;

    motor(int p,int e,int d,int s,int of){

        pwm_pin = p;
        dir = d;
        enb = e;
        dirSide = s;
        offset = of;
        
      }
    void forward(){
      if(dirSide==0){
        digitalWrite(dir,LOW);
      }
      else if(dirSide==1){
        digitalWrite(dir,HIGH);
      }
      analogWrite(pwm_pin,PWM_VALUE+offset);
      digitalWrite(enb,HIGH);
     }

    void backward(){
      if(dirSide==1){
        digitalWrite(dir,LOW);
      }
      else if(dirSide==0){
        digitalWrite(dir,HIGH);
      }
      analogWrite(pwm_pin,PWM_VALUE+offset);
      digitalWrite(enb,HIGH);
     }
    void motor_stop(){
        digitalWrite(enb,LOW);
      }
  };

void Left(motor m1, motor m2,motor m3, motor m4){
    m1.forward();
    m3.forward();
    m2.backward();
    m4.backward();
  }
void Right(motor m1, motor m2,motor m3, motor m4){
    m2.forward();
    m4.forward();
    m1.backward();
    m3.backward();
  }

motor motor1 = motor(PWM1,ENB1,DIR1,0,20);  // 40
motor motor2 = motor(PWM2,ENB2,DIR2,1,20);  // 36
//motor motor3 = motor(PWM3,ENB3,DIR3,0,20);  //53
//motor motor4 = motor(PWM4,ENB4,DIR4,1,20);  //40
  
// we could use nThrottleIn = 0 in loop instead of a separate variable, but using bNewThrottleSignal to indicate we have a new signal 
// is clearer for this first example

void setup()
{
  pinMode(ENB1,OUTPUT);
  pinMode(ENB2,OUTPUT);
  pinMode(ENB3,OUTPUT);
  pinMode(ENB4,OUTPUT);
  pinMode(DIR1,OUTPUT);
  pinMode(DIR2,OUTPUT);
  pinMode(DIR3,OUTPUT);
  pinMode(DIR4,OUTPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(PWM3,OUTPUT);
  pinMode(PWM4,OUTPUT);

  //set PWM frequency of (6, 7, 8, 9. 10)) pins to 4kHz i.e. timer 2 and 4. 
  int myEraser = 7;             // this is 111 in binary and is used as an eraser
  TCCR2B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR2B to 0
  TCCR4B &= ~myEraser;   // this operation (AND plus NOT),  set the three bits in TCCR4B to 0


  int myPrescaler = 2;         // this could be a number in [1 , 6]. In this case, 2 which gives 4000Hz and corresponds in binary to 010.   
  TCCR2B |= myPrescaler;  //this operation (OR), replaces the last three bits in TCCR2B with our new value 010
  TCCR4B |= myPrescaler;  //this operation (OR), replaces the last three bits in TCCR4B with our new value 010
  
  // tell the Arduino we want the function calcInput to be called whenever INT0 (digital pin 2) changes from HIGH to LOW or LOW to HIGH
  // catching these changes will allow us to calculate how long the input pulse is
  attachInterrupt(THROTTLE_SIGNAL_IN,calcInput,CHANGE);
  attachInterrupt(LR_SIGNAL_IN,calcInputLR,CHANGE);

  Serial.begin(9600); 
}

void loop()
{
 // if a new throttle signal has been measured, lets print the value to serial, if not our code could carry on with some other processing
 if(bNewThrottleSignal)
 {
   //Serial.print("Throttle: ");
   //Serial.println(nThrottleIn);  
   if(nThrottleIn>1900){
      Serial.println("Forward: ");
      motor1.forward();
      motor2.forward();
      //motor3.forward();
      //motor4.forward();
    }
   else if (nThrottleIn<1000){
      Serial.println("Backward: ");
      motor1.motor_stop();
      motor2.motor_stop();      
//      motor1.backward();
//      motor2.backward();
      //motor3.backward();
      //motor4.backward();
    }

   // set this back to false when we have finished
   // with nThrottleIn, while true, calcInput will not update
   // nThrottleIn
   bNewThrottleSignal = false;
 }

 if(bNewLRSignal)
 {  
   if(nLRIn<1020){
      Serial.println("Left: ");
      
    }
   else if(nLRIn>1900){
      Serial.println("Right: ");
    }
//   else{
//      motor1.motor_stop();
//      motor2.motor_stop();
////      motor3.motor_stop();
////      motor4.motor_stop();
//    }
    
   // set this back to false when we have finished
   // with nThrottleIn, while true, calcInput will not update
   // nThrottleIn
   bNewLRSignal = false;
 }


 // other processing ...

 
// motor1.backward();
// motor2.backward();
// motor3.backward();
// motor4.backward();
 
 //Left(motor1,motor2,motor3,motor4); 
}

void calcInput()
{
  // if the pin is high, its the start of an interrupt
  if(digitalRead(THROTTLE_SIGNAL_IN_PIN) == HIGH)
  { 
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its 
    // easy to understand and works very well
    ulStartPeriod = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the 
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriod && (bNewThrottleSignal == false))
    {
      nThrottleIn = (int)(micros() - ulStartPeriod);
      ulStartPeriod = 0;

      // tell loop we have a new signal on the throttle channel
      // we will not update nThrottleIn until loop sets
      // bNewThrottleSignal back to false
      bNewThrottleSignal = true;
    }
  }
}

void calcInputLR()
{
  // if the pin is high, its the start of an interrupt
  if(digitalRead(LR_SIGNAL_IN_PIN) == HIGH)
  { 
    // get the time using micros - when our code gets really busy this will become inaccurate, but for the current application its 
    // easy to understand and works very well
    ulStartPeriodLR = micros();
  }
  else
  {
    // if the pin is low, its the falling edge of the pulse so now we can calculate the pulse duration by subtracting the 
    // start time ulStartPeriod from the current time returned by micros()
    if(ulStartPeriodLR && (bNewLRSignal == false))
    {
      nLRIn = (int)(micros() - ulStartPeriodLR);
      ulStartPeriodLR = 0;

      // tell loop we have a new signal on the throttle channel
      // we will not update nThrottleIn until loop sets
      // bNewThrottleSignal back to false
      bNewLRSignal = true;
    }
  }
}
