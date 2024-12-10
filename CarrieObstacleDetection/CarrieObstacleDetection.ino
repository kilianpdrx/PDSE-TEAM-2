#include <Wire.h>
#include <Carrie.h>
const int trig1 = 4;
const int echo1 = 5;
const int trig2 = 6;
const int echo2 = 7;
const int LED = 13;
const int buzzer = A5;

const int motor1Pin1 = 3; //front
const int motor1Pin2 = 9; //back
//Motor 2
const int motor2Pin1 = 10; //back
const int motor2Pin2 = 11; //front

Carrie Prototype(motor1Pin1, motor1Pin2, motor2Pin2, motor2Pin1, trig1, trig2, echo1, echo2, LED, buzzer);

void setup() {
  // put your setup code here, to run once:
  Prototype.setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  Prototype.LED_on_off();

  if(Prototype.getObstacle() == true){
    if(Prototype.getBackwards() == false){
      delay(50);
      Prototype.goBackwards();
    }
    delay(50);
    Prototype.turn();
    delay(50);
    Prototype.obstacleDetection();
  }
  else{
    delay(50);
    Prototype.goForward();
    delay(50);
    Prototype.obstacleDetection();
  }
}
