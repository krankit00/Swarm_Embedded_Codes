#define degconvert 57.2957786 

#include "TimerOne.h"

uint32_t timer;

const byte interruptEncoderPinA = 2;
const byte interruptEncoderPinB = 3;
const byte interruptEncoderPinC = 18;
const byte interruptEncoderPinD = 19;
volatile int ext_counterA=0;
volatile int ext_counterB=0;
volatile float rpm1=0.0;
volatile float rpm2=0.0;

void setup() {
  // put your setup code here, to run once

  //Set pin input and pulldown it.
  pinMode(interruptEncoderPinA, INPUT);
  digitalWrite(interruptEncoderPinA, LOW);
  pinMode(interruptEncoderPinB, INPUT);
  digitalWrite(interruptEncoderPinB, LOW);

  //Initialise the Timer and attach Interrupt.
  Timer1.initialize(50000);
  Timer1.attachInterrupt(calculateRPM);
  
  //Attach Interrupts to the encoders input.
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinA),increase_CounterA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinB),increase_CounterA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinC),increase_CounterB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinD),increase_CounterB, CHANGE);

  Serial.begin(115200);
  delay(100);

  //start a timer
  timer = micros();
}

void loop() {
  
}

void increase_CounterA() {
  ext_counterA++;
}

void increase_CounterB(){
  ext_counterB++;
}

void calculateRPM(){
  rpm1=(ext_counterA*60)/(6.4*19);
    
  rpm2=(ext_counterB*60)/(6.4*19);
  
  ext_counterA=0;
  ext_counterB=0;
  
  Serial.print("RPM1 ");Serial.print(rpm1);
  Serial.print(" RPM2 ");Serial.println(rpm2);
}
