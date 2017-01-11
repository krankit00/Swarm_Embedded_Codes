#include <PID_v1.h>

#include <Wire.h>
#include "TimerOne.h"

#define address 0x1E //0011110b, I2C 7bit address of HMC5883


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
const double kpA=4.0;
const double kiA=0.06;
const double kdA=0.5;

const double kpB=4.0;
const double kiB=0.06;
const double kdB=0.5;


//Definations for rpm
const double max_rpmA=301.0;
volatile double set_rpmA=80.0;
volatile double curr_rpmA=0;

const double max_rpmB=307.0;
volatile double set_rpmB=80.0;
volatile double curr_rpmB=0;

volatile double PWMA;
volatile double PWMB;

PID PID_MotorA(&curr_rpmA, &PWMA, &set_rpmA,kpA,kiA,kdA, DIRECT);
PID PID_MotorB(&curr_rpmB, &PWMB, &set_rpmB,kpB,kiB,kdB, DIRECT);


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
  int x,y,z; //triple axis data

  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }   
  return x;
}

void setup() {
  /* MAGNETOMETER SETUP */
  
  //Initialize Serial and I2C communications
  Serial.begin(115200);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  /* MOTOR AND ENCODER SETUP */

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

  /* PID SETUP */
  PID_MotorA.SetMode(AUTOMATIC);
  PID_MotorB.SetMode(AUTOMATIC);
  
}

void loop() {
  
  PID_MotorA.Compute();
  PID_MotorB.Compute();
  analogWrite(PWMOutputMotorA,PWMA);
  analogWrite(PWMOutputMotorB,PWMB);

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
