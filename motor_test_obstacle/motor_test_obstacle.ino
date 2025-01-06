#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <PID_v1_bc.h>
#include <TimerOne.h>
#define Buffersize 5


// Configuration du Bluetooth (SoftwareSerial sur les pins 10 et 11)
SoftwareSerial HC05(13, 9);  // HC-05 TX Pin, HC-05 RX Pin


// variables for the app
String readBuffer = "";
float valueX, valueY;
bool bouton;
float delta = 0.2;
const float milieu = 2.5;
int manual = 0;
int stop_all = 0;

// variables for the RP
float prof = -1;    // Profondeur
float mdist = -1;   // Distance X
int tracking = -1;  // Suivi actif




elapsedMillis printTime;
// Define robot parameters
const float wheelbase = 0.4;         // Distance between wheels in meters
const double wheel_diameter = 0.14;  //Wheel size
/**************************************************************************************************************************/
// Obstacle Detection Parameters
float distance1, duration1, distance2, duration2;
bool obstacle = false;
/**************************************************************************************************************************/
// Driver Connection Pins (constant current, step/direction bipolar motor driver)
const int dirPin_left = 11, stepPin_left = 12;
const int ena_left = 10;
const int dirPin_right = 5, stepPin_right = 4;
const int ena_right = 6;
// Creates an instance - works for (Bipolar, constant current, step/direction driver)
AccelStepper leftwheel(AccelStepper::DRIVER, stepPin_left, dirPin_left);
AccelStepper rightwheel(AccelStepper::DRIVER, stepPin_right, dirPin_right);  //current position set as 0, enable signal
/**************************************************************************************************************************/
// Sensor connection Pins
const int trig1 = 8;
const int echo1 = 7;

//const int trig2 = 6;
//const int echo2 = 7;

bool light = false;
const int buzzer = 2;
/**************************************************************************************************************************/
bool obstacleDetection();
/**************************************************************************************************************************/
//elapsedMillis printTime;

const long default_maxSpeedLimit = 300;  // Max speed in step/s  0.44209706 * 400 / wheel_diameter
const long default_Acceleration = 200;   //848.8                      // step/s^2 = 1m/s^2

void InitialMoveTest(long trialmove);
void disableMotors();
void enableMotors();
void moveMotors(long targetPositionL, long targetPositionR);
void Motorsrun();
void setSpeedAcceleration(long MyspeedL, long MyAcclL, long MyspeedR, long MyAcclR);
bool initialTestDone = false;
bool motorenable = true;
/**************************************************************************************************************************/

// Variables for Linear Speed PID
double distance_error, linear_speed;  // Linear speed in m/s
double distance_setpoint = 0;         // Desired distance from the target in meters
const float SAFE_DISTANCE_MAX = 3.0;  // Maximum safe distance (in meters) without lost connection
const float SMOOTHING_FACTOR = 0.5;   // Factor for smooth acceleration
const float SAFE_DISTANCE_MIN = 1.0;
// Variables for Angular Speed PID
double theta_error, angular_speed;  // Angular speed in radians/sec
double theta_setpoint;              // Desired angular alignment

long R_Acceleration, L_Acceleration;
double Stepps_R, Stepps_L;  // Right and left wheel speeds
// PID Parameters for Linear Speed

/*************************************************************************************************************************/
void process_app();
void process_RP();


/**************************************************************************************************************************/
unsigned long lastPIDTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastRaspTime = 0;

float inputBuffer[Buffersize]; // buffer size
int bufferindex = 0;

int smoothInput (float newInput)
{
  inputBuffer[bufferindex] = newInput;
  bufferindex = (bufferindex +1)% Buffersize;

  float sum = 0;
  for (int i = 0; i < Buffersize; i++){
    sum += inputBuffer [i];
  }
  return sum / Buffersize;
}

void setup() {

  Serial.begin(9600);
  HC05.begin(9600);


  rightwheel.setPinsInverted(true, false, false);
  leftwheel.setPinsInverted(false, false, false);
  setSpeedAcceleration(default_maxSpeedLimit, default_Acceleration, default_maxSpeedLimit, default_Acceleration);
  pinMode(ena_left, OUTPUT);  // Enable the driver (LOW = enabled), can be used to restart the driver
  digitalWrite(ena_left, LOW);
  pinMode(ena_right, OUTPUT);
  digitalWrite(ena_right, LOW);


 // Timer1.initialize(800);  //trigger ever microsecond
  //Timer1.attachInterrupt(Motorsrun);

  /**************************************************************************************************************************/
  //sensor setups
  /**************************************************************************************************************************/
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  //pinMode(trig2, OUTPUT);
  //pinMode(echo2, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buzzer, OUTPUT);

  Stepps_L = 500;
  Stepps_R = 500;
  R_Acceleration = 300; 
  L_Acceleration = 300;

}

float Xdist = 0, Ydist = 1;  // in m
int track = 0;                 //
float speedfactor = 305 / 300;








void loop() {
    
  // raspberry_talk();
  raspberry_talk();
  obstacle = obstacleDetection();

  
  if(obstacle == 1){
    setSpeedAcceleration(0, 0, 0, 0);
  }
  else{
    if (track == 1){ // automatic mode 

      if (Ydist <= SAFE_DISTANCE_MIN) {
        setSpeedAcceleration(0, 0, 0, 0);
        disableMotors();

      }

      else if (Ydist > SAFE_DISTANCE_MAX) {
        enableMotors();
        linear_speed = 500;
        angular_speed = atan2(Xdist, Ydist) * 3;
        Stepps_R = linear_speed + (angular_speed * 50); //wheelbase / 2.0 * 400) / wheel_diameter / (PI);  // Right wheel rotation speed (rad/s) ~170*angular_speed
        Stepps_R = Stepps_R * speedfactor;
        Stepps_L = linear_speed - (angular_speed * 50); //wheelbase / 2.0 * 400) / wheel_diameter / (PI);  // Left wheel rotation speed (rad/s)
        setSpeedAcceleration(Stepps_L, 100, Stepps_R, 100);
      }

      else{
        enableMotors();
        linear_speed = (Ydist - 1) * 200;
        angular_speed = atan2(Xdist, Ydist) * 0.3;
        Stepps_R = linear_speed + (angular_speed * 50); //wheelbase / 2.0 * 400) / wheel_diameter / (PI);  // Right wheel rotation speed (rad/s)
        Stepps_R = Stepps_R * speedfactor;
        Stepps_L = linear_speed - (angular_speed * 50);//wheelbase / 2.0 * 400) / wheel_diameter / (PI);  // Left wheel rotation speed (rad/s)
        setSpeedAcceleration(Stepps_L, 100, Stepps_R, 100);
      }

      for (int i=0;i<1000;i++){
        moveMotors(10000, 10000);
        rightwheel.run();
        leftwheel.run();
      }
    }
    else {
      setSpeedAcceleration(0, 0, 0, 0);
      disableMotors();
    }
  }
  delay(10);

}



  void setSpeedAcceleration(long MyspeedL, long MyAcclL, long MyspeedR, long MyAcclR) {
  leftwheel.setMaxSpeed(MyspeedL);     // set the maximum speed and initial speed.
  leftwheel.setAcceleration(MyAcclL);  //This is an expensive call since it requires a square root to be calculated. Don't call more often than needed
  rightwheel.setMaxSpeed(MyspeedR);
  rightwheel.setAcceleration(MyAcclR);
}

  void moveMotors(long targetPositionL, long targetPositionR) {
    rightwheel.move(targetPositionR);  // Set the target position, also recalculate the speed
    leftwheel.move(targetPositionL);
  }
  void Motorsrun() {
    rightwheel.run();
    leftwheel.run();
  }

  void InitialMoveTest(long trialmove) {
    moveMotors(trialmove, trialmove);
    for (int i = 0; i < trialmove; i++) {
      rightwheel.run();
      leftwheel.run();
    }
    moveMotors(-trialmove, -trialmove);
    for (int i = 0; i < trialmove; i++) {
      rightwheel.run();
      leftwheel.run();
    }
  }
  void disableMotors()  //emergency stop
  {
    leftwheel.stop();  //based on the current speed and acceleration
    rightwheel.stop();
    leftwheel.disableOutputs();
    rightwheel.disableOutputs();
    digitalWrite(ena_left, HIGH);
    digitalWrite(ena_right, HIGH);
  }

  void enableMotors() {
    leftwheel.enableOutputs();
    rightwheel.enableOutputs();
    digitalWrite(ena_left, LOW);
    digitalWrite(ena_right, LOW);
  }



bool obstacleDetection() {
    digitalWrite(trig1, LOW);
    delayMicroseconds(2);
    digitalWrite(trig1, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig1, LOW);
    duration1 = pulseIn(echo1, HIGH, 6000);
    /**************************************************************
  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  duration2 = pulseIn(echo2, HIGH);
  /**************************************************************/
    distance1 = 0.0343 * duration1 * 0.5;
    // Serial.println(distance1);
    // distance2 = 0.0343*duration2*0.5;
    bool res = false;
  
    if ((smoothInput(distance1) != 0 && smoothInput(distance1) < 60 )/*|| (distance2 != 0 && distance2 < 60)*/) {  //60cm = 3500 us
        res = true;
      }
    else {
      res = false;
    }
    return res;
  }

void raspberry_talk(){
    if (Serial.available() > 0) {
      String data = Serial.readStringUntil('\n');  // Lire une ligne complète

      int firstSeparator = data.indexOf(',');                       // Trouver la première virgule
      int secondSeparator = data.indexOf(',', firstSeparator + 1);  // Trouver la deuxième virgule

      if (firstSeparator != -1 && secondSeparator != -1) {
        prof = data.substring(0, firstSeparator).toFloat();                     // Extraire la première valeur
        mdist = data.substring(firstSeparator + 1, secondSeparator).toFloat();  // Extraire la deuxième valeur
        tracking = data.substring(secondSeparator + 1).toFloat();               // Extraire la troisième valeur

        // Sending back the values to the RP for monitoring
        Serial.print("Profondeur: ");
        Serial.print(prof);
        Serial.print(" m, X_dist: ");
        Serial.print(mdist);
        Serial.print(" m, Tracking: ");
        Serial.println(tracking);

        Xdist = mdist;
        Ydist = prof;
        track = tracking;
      }
    }
  }

void process_RP(){
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Lire une ligne complète
    
    int firstSeparator = data.indexOf(',');    // Trouver la première virgule
    int secondSeparator = data.indexOf(',', firstSeparator + 1); // Trouver la deuxième virgule

    if (firstSeparator != -1 && secondSeparator != -1) {
      prof = data.substring(0, firstSeparator).toFloat(); // Extraire la première valeur
      mdist = data.substring(firstSeparator + 1, secondSeparator).toFloat(); // Extraire la deuxième valeur
      tracking = data.substring(secondSeparator + 1).toInt(); // Extraire la troisième valeur


      
      // Sending back the values to the RP for monitoring
      Serial.print("Profondeur: ");
      Serial.print(prof);
      Serial.print(" m, X_dist: ");
      Serial.print(mdist);
      Serial.print(" m, Tracking: ");
      Serial.println(tracking);

      Xdist= mdist;
      Ydist= prof;
      track = tracking;
    }
  }
}

