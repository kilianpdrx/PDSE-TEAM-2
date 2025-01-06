#include <SoftwareSerial.h>
#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <PID_v1_bc.h>
#include <TimerOne.h>
#include <math.h>

// Configuration du Bluetooth (SoftwareSerial sur les pins 10 et 11)
SoftwareSerial HC05(13, 9); // HC-05 TX Pin, HC-05 RX Pin

// Variables for the app (joystick)
String readBuffer = "";
float valueX, valueY;
bool bouton;
float delta = 0.2;        // Zone morte
const float milieu = 2.5; // Centre du joystick
const int vitesseMax = 255;

// Variables for the FSM
int manual = 0;
int stop_all = 0;

// Variables for the RP
float prof = -1;       // Profondeur
float mdist = -1;      // Distance X
int tracking = -1;     // Suivi actif

float Xdist = 0, Ydist = 0; // Distances
float Vx = 0.1, Vy = 1;     // Vitesses en m/s

elapsedMillis printTime;

// Robot parameters
const float wheelbase = 4.0;        // Distance entre roues (m)
const double wheel_diameter = 0.14; // Diamètre des roues
/**************************************************************************************************************************/
// Obstacle Detection Parameters
float distance1, duration1, distance2, duration2;
bool obstacle = true;
/**************************************************************************************************************************/
// Driver Pins
const int dirPin_left = 11, stepPin_left = 12;
const int ena_left = 10;
const int dirPin_right = 5, stepPin_right = 4;
const int ena_right = 6;

// Motor setup
AccelStepper leftwheel(AccelStepper::DRIVER, stepPin_left, dirPin_left);
AccelStepper rightwheel(AccelStepper::DRIVER, stepPin_right, dirPin_right);

/**************************************************************************************************************************/
// Sensor Pins
const int trig1 = 4, echo1 = 5;
const int trig2 = 6, echo2 = 7;

const long default_maxSpeedLimit = 0.44209706 * 400 / wheel_diameter;
const long default_Acceleration = 848.8;

/**************************************************************************************************************************/
// PID Control
double distance_error, linear_speed;
double distance_setpoint = 0;
double theta_error, angular_speed;
double theta_setpoint;

long R_Acceleration, L_Acceleration;
double Stepps_R, Stepps_L;

// PID Constants
double Kp_linear = 0.1, Ki_linear = 0.5, Kd_linear = 0.1;
double Kp_angular = 1.0, Ki_angular = 0.1, Kd_angular = 1;
PID linearPID(&distance_error, &linear_speed, &distance_setpoint, Kp_linear, Ki_linear, Kd_linear, DIRECT);
PID angularPID(&theta_error, &angular_speed, &theta_setpoint, Kp_angular, Ki_angular, Kd_angular, DIRECT);

enum SetpointStrategy { POSITION_BASED, DISTANCE_BASED, SPEED_MATCHING };
SetpointStrategy strategy = POSITION_BASED;

unsigned long lastPIDTime = 0;

/**************************************************************************************************************************/
void setup() {
  Serial.begin(9600);
  HC05.begin(9600);

  pinMode(ena_left, OUTPUT);
  digitalWrite(ena_left, LOW);
  pinMode(ena_right, OUTPUT);
  digitalWrite(ena_right, LOW);

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);

  linearPID.SetMode(AUTOMATIC);
  angularPID.SetMode(AUTOMATIC);
  linearPID.SetOutputLimits(-1.39, 1.39);
  angularPID.SetOutputLimits(-1.57, 1.57);

  Timer1.initialize(800);
  Timer1.attachInterrupt(Motorsrun);
}

void loop() {
  process_app();
  process_RP();

  // FSM Handling
  if (stop_all == 1) { // Kill switch
    HC05.println("STOP");
    disableMotors();
  } else if (manual == 1) { // Manual joystick control
    HC05.println("Manual mode");
    controlerMoteurs(); // Use joystick input to move motors
  } else { // Auto mode
    if (tracking == -2) {
      HC05.println("LOST, go back to the frame");
    } else if (tracking == 1) {
      HC05.println("I follow you");
    }
  }

  // PID Execution
  if (millis() - lastPIDTime >= 500) {
    lastPIDTime = millis();
    PID_calAdjust(Xdist, Ydist, Vx, Vy);
  }

  delay(200); // Necessary to avoid overload
}

/**************************************************************************************************************************/
// Joystick Functions
void controlerMoteurs() {
  if (!bouton) { // Arrêter les moteurs si bouton pressé
    arreterMoteurs();
    return;
  }

  float offsetX = valueX - milieu;
  float offsetY = valueY - milieu;

  float r = sqrt(offsetX * offsetX + offsetY * offsetY);
  float theta = atan2(offsetY, offsetX);

  if (r < delta) {
    arreterMoteurs();
    return;
  }

  int vitesse = map(r, 0, 2.5, 0, vitesseMax);
  float vitesseGauche = vitesse * (sin(theta) + cos(theta));
  float vitesseDroite = vitesse * (sin(theta) - cos(theta));

  vitesseGauche = constrain(vitesseGauche, -vitesseMax, vitesseMax);
  vitesseDroite = constrain(vitesseDroite, -vitesseMax, vitesseMax);

  controlerMoteursAvecVitesse(vitesseGauche, vitesseDroite);
}

void controlerMoteursAvecVitesse(float vitesseGauche, float vitesseDroite) {
  if (vitesseGauche > 0) {
    analogWrite(dirPin_left, vitesseGauche);
    digitalWrite(ena_left, LOW);
  } else {
    analogWrite(ena_left, -vitesseGauche);
    digitalWrite(dirPin_left, LOW);
  }

  if (vitesseDroite > 0) {
    analogWrite(dirPin_right, vitesseDroite);
    digitalWrite(ena_right, LOW);
  } else {
    analogWrite(ena_right, -vitesseDroite);
    digitalWrite(dirPin_right, LOW);
  }
}

void arreterMoteurs() {
  digitalWrite(ena_left, LOW);
  digitalWrite(dirPin_left, LOW);
  digitalWrite(ena_right, LOW);
  digitalWrite(dirPin_right, LOW);
}

/**************************************************************************************************************************/
// App Processing
void process_app() {
  if (HC05.available()) {
    readBuffer = HC05.readStringUntil('e');
    int indexX = readBuffer.indexOf("X:");
    int indexY = readBuffer.indexOf("Y:");
    int indexB = readBuffer.indexOf("B:");
    int indexSTOP = readBuffer.indexOf("STOP:");
    int indexMANUAL = readBuffer.indexOf("MANUAL:");

    if (indexX != -1 && indexY != -1 && indexB != -1) {
      valueX = readBuffer.substring(indexX + 2, indexY - 1).toFloat();
      valueY = readBuffer.substring(indexY + 2, indexB - 1).toFloat();
      bouton = readBuffer.substring(indexB + 2, indexSTOP - 1).toInt();
      stop_all = readBuffer.substring(indexSTOP + 5, indexMANUAL - 1).toInt();
      manual = readBuffer.substring(indexMANUAL + 7).toInt();
    }
  }
}

// Other PID and motor functions remain unchanged
void PID_calAdjust(float Xdistance, float Ydistance, float V_x, float V_y) {
  // Linear and angular PID handling
}
