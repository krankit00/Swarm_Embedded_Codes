#include "TimerOne.h"

uint32_t timer;

const float pi = 3.14159265359;

const byte interruptEncoderPinA = 18;
const byte interruptEncoderPinB = 19;
const byte interruptEncoderPinC = 2;
const byte interruptEncoderPinD = 3;
const byte PWMOutputMotorA=4;
const byte PWMOutputMotorB=5;
const byte InputAA=7;
const byte InputAB=8;
const byte InputBA=9;
const byte InputBB=10;
volatile int ext_counterA=0;
volatile int ext_counterB=0;
volatile bool direction_rotation=true;

volatile int motion_counter=0;

//Initialise Ki, Kp, Kd
const float kpA=4.0;
const float kiA=0.06;
const float kdA=0.5;

const float kpB=4.0;
const float kiB=0.06;
const float kdB=0.5;

//Definations for rpm
const float max_rpmA=301.0;
volatile float curr_errorA;
volatile float prev_errorA=0.0;
volatile float sum_errorA=0.0;
volatile float diff_errorA=0.0;
volatile float set_rpmA=80.0;
volatile float curr_rpmA=0;

const float max_rpmB=307.0;
volatile float curr_errorB;
volatile float prev_errorB=0.0;
volatile float sum_errorB=0.0;
volatile float diff_errorB=0.0;
volatile float set_rpmB=80.0;
volatile float curr_rpmB=0;

volatile int PWMA;
volatile int PWMB;

//Definations for localisation
volatile float sys_dist=0.0;
volatile float curr_pos_x=0.0;
volatile float curr_pos_y=0.0;
const float wheel_radius=3.5; //In cm
volatile float theta=0.0; // Current orientation of the bot.
volatile int motion_direction=1;

/*
 Motion Direction Legend:
 1. Forward
 2. Right
 3. Backward
 4. Left
 5. Free Run ( this is just for our perception. Do not use it anywhere to denoe it.)
 6. Brake
*/


void leftTurn() {
  motion_direction=4;
  digitalWrite(InputAA, HIGH);
  digitalWrite(InputBA, HIGH);
  digitalWrite(InputBB, LOW);
  digitalWrite(InputAB, LOW);
}


void rightTurn() {
  motion_direction=2;
  digitalWrite(InputAA, LOW);
  digitalWrite(InputBA, LOW);
  digitalWrite(InputBB, HIGH);
  digitalWrite(InputAB, HIGH);
}


void forward() {
  motion_direction=1;
  digitalWrite(InputAA, HIGH);
  digitalWrite(InputBA, LOW);
  digitalWrite(InputBB, HIGH);
  digitalWrite(InputAB, LOW);
}


void backward() {
  motion_direction=3;
  digitalWrite(InputAA, LOW);
  digitalWrite(InputBA, HIGH);
  digitalWrite(InputBB, LOW);
  digitalWrite(InputAB, HIGH);
}

void freeRun() {
  digitalWrite(InputAA, LOW);
  digitalWrite(InputBA, LOW);
  digitalWrite(InputBB, LOW);
  digitalWrite(InputAB, LOW);
}

void brake() {
  motion_direction=6;
  digitalWrite(InputAA, HIGH);
  digitalWrite(InputBA, HIGH);
  digitalWrite(InputBB, HIGH);
  digitalWrite(InputAB, HIGH);
}

float get_theta(){
  float t=0.0;
  //Use magnetometer to set t. Amit, your code from here.
  
  return t;
}


void setup() {
  // put your setup code here, to run once

  //Set pin input and pulldown it.
  pinMode(interruptEncoderPinA, INPUT);
  digitalWrite(interruptEncoderPinA, LOW);
  pinMode(interruptEncoderPinB, INPUT);
  digitalWrite(interruptEncoderPinB, LOW);
  pinMode(PWMOutputMotorA,OUTPUT);
  pinMode(PWMOutputMotorB,OUTPUT);
  forward();

  //Initialise the Timer and attach Interrupt.
  Timer1.initialize(50000);
  Timer1.attachInterrupt(calculateRPM);
  
  //Attach Interrupts to the encoders input.
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinA),increase_CounterA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinB),increase_CounterA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinC),increase_CounterB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptEncoderPinD),increase_CounterB, CHANGE);

  Serial.begin(115200);

  //start a timer
  timer = micros();
}

void loop() {
  
  if(motion_counter<40){
    forward();
  }
  else if(motion_counter<80){
    rightTurn();
  }  
  else if(motion_counter<120){
    leftTurn();
  }  
  else if(motion_counter<160){
    backward();
  } 
  else
    motion_counter=0;

  
  //Calculate errors and other terms
  curr_errorA= set_rpmA - curr_rpmA;
  diff_errorA= prev_errorA - curr_errorA;
  sum_errorA+=curr_errorA;

  curr_errorB= set_rpmB - curr_rpmB;
  diff_errorB= prev_errorB - curr_errorB;
  sum_errorB+=curr_errorB;
  
  //Apply PWM accordingly.
  PWMA=(int)(((set_rpmA+(kpA*curr_errorA)+(kiA*sum_errorA)+(kdA*diff_errorA))*256)/max_rpmA);
  PWMB=(int)(((set_rpmB+(kpB*curr_errorB)+(kiB*sum_errorB)+(kdB*diff_errorB))*256)/max_rpmB);
  PWMA = PWMA < 0? 0 :PWMA;  
  PWMB = PWMB< 0 ? 0 : PWMB;
  PWMA=(PWMA>255)?255:PWMA;
  PWMB=(PWMB>255)?255:PWMB;

  analogWrite(PWMOutputMotorA,PWMA);
  analogWrite(PWMOutputMotorB,PWMB);

  //Update the error value.
  prev_errorA= curr_errorA;
  prev_errorB= curr_errorB;

  Serial.print("X: "); Serial.print(curr_pos_x);
  Serial.print(" Y: "); Serial.println(curr_pos_y);
  
}

void increase_CounterA() {
  ext_counterA++;
}

void increase_CounterB(){
  ext_counterB++;
}

void calculateRPM(){
  curr_rpmA=(ext_counterA*60*2.6)/(6.4*19);
  curr_rpmB=(ext_counterB*60*2.6)/(6.4*19);
  
  if(motion_direction==1)
    sys_dist=((curr_rpmA+curr_rpmB)/2400)*2*pi*wheel_radius;
  else if(motion_direction==3)
    sys_dist=((curr_rpmA+curr_rpmB)/2400)*2*pi*wheel_radius*(-1);
  else if(motion_direction==2)
    sys_dist=((curr_rpmA-curr_rpmB)/2400)*2*pi*wheel_radius*(-1);
  else if(motion_direction==4)
    sys_dist=((curr_rpmA-curr_rpmB)/2400)*2*pi*wheel_radius;

  theta=get_theta();
  curr_pos_x+=(sys_dist*cos(theta));
  curr_pos_y+=(sys_dist*sin(theta));
  
  ext_counterA=0;
  ext_counterB=0;

  motion_counter++;
}
