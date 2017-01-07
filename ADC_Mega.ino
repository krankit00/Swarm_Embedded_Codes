
int sensorValue;
int sensorPin[6]= {A0,A1,A2,A3,A4,A5};

void setup() {
  // put your setup code here, to run once:

    Serial.begin(9600); // begin sending over serial port
  // declaration of pin modes
  analogReference(DEFAULT);
  
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  int i;
  
 


}

void loop() {
  
  // put your main code here, to run repeatedly:
for(int i = 0; i < 6; i++){
   // read the value from the sensor:
    sensorValue = analogRead(sensorPin[i]);
    Serial.print("sensor_i");
    Serial.println(sensorValue);
}
  
}


