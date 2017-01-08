int rpm=64;
#define in1 7  //+12
#define in2 8  //0
#define in3 9  //+12
#define in4 10 //0
//Yellow = Power
//Blue = Ground

void setup() {
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
}

void leftTurn() {
  Serial.println("Left");
  analogWrite(in1,rpm);
  analogWrite(in2,0);
  analogWrite(in3,rpm);
  analogWrite(in4,0);
}


void rightTurn() {
  Serial.println("Right");
  analogWrite(in1,0);
  analogWrite(in2,rpm);
  analogWrite(in3,0);
  analogWrite(in4,rpm);
}


void forward() {
  Serial.println("Forward");
  analogWrite(in1,rpm);
  analogWrite(in2,0);
  analogWrite(in3,0);
  analogWrite(in4,rpm);
}


void backward() {
  Serial.println("Backward");
  analogWrite(in1,0);
  analogWrite(in2,rpm);
  analogWrite(in3,rpm);
  analogWrite(in4,0);
}

void freeRun() {
  Serial.println("Free Run");
  analogWrite(in1,0);
  analogWrite(in2,0);
  analogWrite(in3,0);
  analogWrite(in4,0);
}

void brake() {
  Serial.println("Brake");
  analogWrite(in1,rpm);
  analogWrite(in2,rpm);
  analogWrite(in3,rpm);
  analogWrite(in4,rpm);
}

void loop() {
  freeRun();
  delay(2000);
  leftTurn();
  delay(2000);
  rightTurn();
  delay(2000);
  forward();
  delay(2000);
  backward();
  delay(2000);
  brake();
  delay(2000);
  freeRun();// put your main code here, to run repeatedly:
}
