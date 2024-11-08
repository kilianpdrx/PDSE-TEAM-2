//Motor 1
const int motorPin1 = 9;
const int motorPin2 = 8; 

int speed = 5000;


void setup(){



  //Set pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);


  //Motor Control A in both directions
  analogWrite(motorPin1, speed);
  delay(2000);
  analogWrite(motorPin1, 100);
  delay(2000);
  analogWrite(motorPin1, 0);
  delay(200);
  analogWrite(motorPin2, speed);
  delay(2000);
  analogWrite(motorPin2, 0);

}

void loop(){

}