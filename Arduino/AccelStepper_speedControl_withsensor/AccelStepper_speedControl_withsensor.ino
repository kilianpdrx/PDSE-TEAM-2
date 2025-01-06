/*     The controller sets the delay between steps, you may notice the motor is less responsive to changes in the sensor value at low speeds.
 */
#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <PID_v1_bc.h>
#include <TimerOne.h>

float prof = -1;
float mdist = -1;
int tracking = -1;

elapsedMillis printTime;
// Define robot parameters
const float wheelbase = 4.0;        // Distance between wheels in meters
const double wheel_diameter = 0.14;  //Wheel size
/**************************************************************************************************************************/
// Obstacle Detection Parameters
float distance1, duration1, distance2, duration2;
bool obstacle = true;
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
const int trig1 = 4;
const int echo1 = 5;

const int trig2 = 6;
const int echo2 = 7;

bool light = false;
/**************************************************************************************************************************/
//Raspberry Connection Pins

/**************************************************************************************************************************/
//elapsedMillis printTime;

const long default_maxSpeedLimit = 0.44209706 * 400 / wheel_diameter;  // Max speed in step/s
const long default_Acceleration = 848.8;                         // step/s^2 = 1m/s^2

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
double distance_setpoint = 0;       // Desired distance from the target in meters
const float SAFE_DISTANCE_MAX = 3.0;  // Maximum safe distance (in meters) without lost connection
const float SMOOTHING_FACTOR = 0.5;   // Factor for smooth acceleration
// Variables for Angular Speed PID
double theta_error, angular_speed;  // Angular speed in radians/sec
double theta_setpoint;              // Desired angular alignment

long R_Acceleration, L_Acceleration;
double Stepps_R, Stepps_L;  // Right and left wheel speeds
// PID Parameters for Linear Speed
double Kp_linear = 0.1, Ki_linear = 0.5, Kd_linear = 0.1;
double Kp_angular = 1.0, Ki_angular = 0.1, Kd_angular = 1;
/*************************************************************************************************************************/
PID linearPID(&distance_error, &linear_speed, &distance_setpoint, Kp_linear, Ki_linear, Kd_linear, DIRECT);
PID angularPID(&theta_error, &angular_speed, &theta_setpoint, Kp_angular, Ki_angular, Kd_angular, DIRECT);

// Setpoint Strategy Selector
enum SetpointStrategy { POSITION_BASED,
                        DISTANCE_BASED,
                        SPEED_MATCHING };

SetpointStrategy strategy = POSITION_BASED;  // Change this to switch strategies

void PID_calAdjust(float Xdistance, float Ydistance, float V_x, float V_y);

/**************************************************************************************************************************/
unsigned long lastPIDTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastRaspTime = 0;

void setup() {

  setSpeedAcceleration(default_maxSpeedLimit, default_Acceleration, default_maxSpeedLimit, default_Acceleration);
  pinMode(ena_left, OUTPUT);  // Enable the driver (LOW = enabled), can be used to restart the driver
  digitalWrite(ena_left, LOW);
  pinMode(ena_right, OUTPUT);
  digitalWrite(ena_right, LOW);

  linearPID.SetMode(AUTOMATIC);  // Initialize PIDs
  angularPID.SetMode(AUTOMATIC);

  linearPID.SetOutputLimits(-1.3889, 1.3889);       // Linear speed (m/s), adjust as per robot's max speed < 5km/h
  angularPID.SetOutputLimits(-1.57, 1.57);  // Angular speed (rad/s), adjust as per robot's max turn rate < 90 degree/s

  Timer1.initialize(800 );        //trigger ever microsecond
  Timer1.attachInterrupt(Motorsrun);

  /**************************************************************************************************************************/
  //sensor setups
  /**************************************************************************************************************************/
  pinMode(trig1, OUTPUT);  
	pinMode(echo1, INPUT);  
  pinMode(trig2, OUTPUT);  
	pinMode(echo2, INPUT);  
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);  //For test display

}

  float Xdist = 2, Ydist = 2;  // in m
  float Vx = 0.1, Vy = 1;        // in m/s
  float Sensordist;

void loop() {

  // Read from external
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Lire une ligne complète
    
    int firstSeparator = data.indexOf(',');    // Trouver la première virgule
    int secondSeparator = data.indexOf(',', firstSeparator + 1); // Trouver la deuxième virgule

    if (firstSeparator != -1 && secondSeparator != -1) {
      prof = data.substring(0, firstSeparator).toFloat(); // Extraire la première valeur
      mdist = data.substring(firstSeparator + 1, secondSeparator).toFloat(); // Extraire la deuxième valeur
      tracking = data.substring(secondSeparator + 1).toFloat(); // Extraire la troisième valeur
      
      // Afficher les valeurs reçues
      Serial.print("Profondeur: ");
      Serial.print(prof);
      Serial.print(" m, X_dist: ");
      Serial.print(mdist);
      Serial.print(" m, Tracking: ");
      Serial.println(tracking);

      Xdist= mdist;
      Ydist= prof;
    }
  }
  
  /**************************************************************************************************************************/
  //Distance sequence generation
  //Xdist = random (0, 1000)/ 1000.0;
  //Ydist = random (0, 3000)/ 1000.0;
/*
   //  Read from Raspberry
    if (millis() - lastRaspTime >= 500) { // Read every 100 ms
        lastRaspTime = millis();

        if(Ydist >= 0){
              Ydist = Ydist - 0.2; 
        }
        Xdist = Xdist - 0.02;

        //readRaspberry();
    }
  
  /**************************************************************************************************************************/
  /**************************************************************************************************************************/
  
  digitalWrite(LED_BUILTIN, light);
  if (!initialTestDone) {  //Initial move
    InitialMoveTest(100);
    initialTestDone = true;
  }
  moveMotors(10000,10000);
  /*  // Prioritize motor control
  if ((obstacle == true || Ydist <= 0.6) && motorenable == true)  //emergency brake
  {
    light = true;
    disableMotors();
    motorenable = false;
  } 
  else if(motorenable == true){
    moveMotors(10000,10000);
  leftwheel.run();
  rightwheel.run();
  }
  else if (motorenable == false && obstacle == false) {
    enableMotors();
    motorenable = true;
    light = false;
  }

*/



  /**************************************************************************************************************************/
  //   Sensor commands
/*
  if (millis() - lastSensorTime >= 1000) {  // Update sensor every 500 ms
    lastSensorTime = millis();

      unsigned long startTime = micros();
    obstacleDetection();
    unsigned long endTime = micros();
    Serial.println("Sensor time:");
    Serial.print(endTime-startTime);
}


/**************************************************************************************************************************/

  // speed control PID
  if (millis() - lastPIDTime >= 500) {  // PID compute every 100 ms
    lastPIDTime = millis();
    unsigned long startTime = micros();
    PID_calAdjust(Xdist, Ydist, Vx, Vy);      //PID running 2-3 ms
    unsigned long endTime = micros();
    Serial.print("PID time:");
    Serial.print(endTime-startTime);
    
  }


/*
  if (printTime >= 1000) {  // print every second for test
    printTime = 0;
    // display the updated speed and acceleration to the motors
    // Debugging output (optional, for monitoring the system)
    Serial.print("Distance: ");
    Serial.print(Xdist);
    Serial.print(", ");
    Serial.print(Ydist);
    Serial.println(" | Car_Speed: ");
    Serial.print(linear_speed);
    Serial.print(" | Turning: ");
    Serial.print(angular_speed);
    Serial.println(" | Acceleration: ");
    Serial.print(L_Acceleration);
    Serial.print(", ");
    Serial.println(R_Acceleration);
  }
/**************************************************************************************************************************/

}

void PID_calAdjust(float Xdistance, float Ydistance, float V_x, float V_y) {
  //Linear speed control
  distance_error = Ydistance - distance_setpoint;  // Error in distance
  linearPID.Compute();                             // Compute linear speed (linear_speed)
  if (Ydistance > SAFE_DISTANCE_MAX) {             // Speedup if too far away
    // Too far: Speed up proportionally
    linear_speed = linear_speed + (V_y + (Ydistance - SAFE_DISTANCE_MAX)) * 0.3;  //factor to change
  } else {
  }

  // Limit the target speed to the maximum speed (m/s)
  if (linear_speed > 1.39) {
    linear_speed = 1.39;
  }

  /**************************************************************************************************************************/
  // Angular Speed Control
  switch (strategy) {
    case POSITION_BASED:
      // Dynamic position-based setpoint: Align to target angle
      theta_setpoint = atan2(Xdistance, Ydistance);
      break;

    case DISTANCE_BASED:
      // Distance-based setpoint: Modulate angular speed by proximity
      theta_setpoint = atan2(Xdistance, Ydistance) / Ydistance;
      break;

    case SPEED_MATCHING:
      // Speed-matching setpoint: Adjust angular alignment based on target speed
      theta_setpoint = atan2(Xdistance, Ydistance) + (V_y - linear_speed) * 0.1;
      break;
  }

  theta_error = theta_setpoint;
  angularPID.Compute();

  // Convert Linear and Angular Speed to Wheel Speeds
  Stepps_R = (linear_speed + angular_speed * wheelbase / 2.0) / wheel_diameter * 60 / (2 *2 * PI);  // Right wheel rotation speed (rad/s)
  Stepps_L = (linear_speed - angular_speed * wheelbase / 2.0) / wheel_diameter * 60 / (2*2* PI);  // Left wheel rotation speed (rad/s)
  // Updating acceleration accordingly

  // Calculate the required acceleration for smooth motion
  R_Acceleration = (Stepps_R - rightwheel.speed()) * SMOOTHING_FACTOR;
  L_Acceleration = (Stepps_L - leftwheel.speed()) * SMOOTHING_FACTOR;
  // Limit the acceleration to the maximum allowed value
  /*if (targetAcceleration > MAX_ACCELERATION) {
    targetAcceleration = MAX_ACCELERATION;
  } else if (targetAcceleration < -MAX_ACCELERATION) {
    targetAcceleration = -MAX_ACCELERATION;
  }*/

  // Update the car's speed and acceleration
  setSpeedAcceleration(Stepps_L, L_Acceleration, Stepps_R, R_Acceleration);
  moveMotors(Stepps_L, Stepps_R);  //excess steps between PID updates, default 10Hz = 100ms
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
void Motorsrun(){
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

void obstacleDetection(){
  /*digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  duration1 = pulseIn(echo1, HIGH);*/

  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  duration2 = pulseIn(echo2, HIGH);

 // distance1 = 0.0343*duration1*0.5;
  distance2 = 0.0343*duration2*0.5;

  if(distance2 < 60 /*|| distance1 < 60*/){
    obstacle = true;
  }
  else{
    obstacle = false;
  }
}