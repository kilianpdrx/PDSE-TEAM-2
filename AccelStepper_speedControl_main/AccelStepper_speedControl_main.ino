/*     The controller sets the delay between steps, you may notice the motor is less responsive to changes in the sensor value at low speeds.
 */
#include <AccelStepper.h>
#include <elapsedMillis.h>
#include <PID_v1_bc.h>

// Define robot parameters
const float wheelbase = 4.0;          // Distance between wheels in meters
const double wheel_radius = 0.075 ;  //Wheel size
/**************************************************************************************************************************/
// Driver Connection Pins (constant current, step/direction bipolar motor driver)
const int dirPin_left = 50, stepPin_left = 48;
const int ena_left = 52;
const int dirPin_right = 24, stepPin_right = 22;
const int ena_right = 26;
// Creates an instance - works for (Bipolar, constant current, step/direction driver)
AccelStepper leftwheel(AccelStepper::DRIVER, stepPin_left, dirPin_left);
AccelStepper rightwheel(AccelStepper::DRIVER, stepPin_right, dirPin_right);  //current position set as 0, enable signal
/**************************************************************************************************************************/
// Sensor connection Pins

/**************************************************************************************************************************/
//Raspberry Connection Pins

/**************************************************************************************************************************/
//elapsedMillis printTime;

const long default_maxSpeedLimit = 1178.9;  // Max speed in step/s
const long default_Acceleration = 848.8;    // step/s^2.

const float SAFE_DISTANCE_MAX = 3.0;  // Maximum safe distance (in meters) without lost connection
const float SMOOTHING_FACTOR = 0.1;   // Factor for smooth acceleration

void InitialMoveTest(long trialmove);
void disableMotors();
void enableMotors();
void moveMotors(long targetPositionL, long targetPositionR);
void setSpeedAcceleration(long MyspeedL, long MyAcclL, long MyspeedR, long MyAcclR);
/**************************************************************************************************************************/

// Variables for Linear Speed PID
double distance_error, linear_speed;  // Linear speed in m/s
double distance_setpoint = 1.5;       // Desired distance from the target in meters

// Variables for Angular Speed PID
double theta_error, angular_speed;  // Angular speed in radians/sec
double theta_setpoint;              // Desired angular alignment

long R_Acceleration, L_Acceleration;
// PID Parameters for Linear Speed
double Kp_linear = 2.0, Ki_linear = 0.5, Kd_linear = 0.1;
PID linearPID(&distance_error, &linear_speed, &distance_setpoint, Kp_linear, Ki_linear, Kd_linear, DIRECT);

// PID Parameters for Angular Speed
double Kp_angular = 1.0, Ki_angular = 0.3, Kd_angular = 0.1;
PID angularPID(&theta_error, &angular_speed, &theta_setpoint, Kp_angular, Ki_angular, Kd_angular, DIRECT);

// Wheel Rotation Speeds (in radians/sec)
double Stepps_R, Stepps_L;  // Right and left wheel speeds

// Setpoint Strategy Selector
enum SetpointStrategy { POSITION_BASED,
                        DISTANCE_BASED,
                        SPEED_MATCHING };
SetpointStrategy strategy = POSITION_BASED;  // Change this to switch strategies

/**************************************************************************************************************************/
void setup() {

  setSpeedAcceleration(default_maxSpeedLimit, default_Acceleration, default_maxSpeedLimit, default_Acceleration);
  pinMode(ena_left, OUTPUT);  // Enable the driver (LOW = enabled), can be used to restart the driver
  digitalWrite(ena_left, LOW);

  pinMode(ena_right, OUTPUT);
  digitalWrite(ena_right, LOW);  // Enable the driver (LOW = enabled)

  InitialMoveTest(20);  //Initial move test

  linearPID.SetMode(AUTOMATIC);  // Initialize PIDs
  angularPID.SetMode(AUTOMATIC);

  linearPID.SetOutputLimits(0, 2.0);      // Linear speed (m/s), adjust as per robot's max speed
  angularPID.SetOutputLimits(-2.0, 2.0);  // Angular speed (rad/s), adjust as per robot's max turn rate

  /**************************************************************************************************************************/
  //sensor setups
  /**************************************************************************************************************************/

  Serial.begin(9600);  //For test display
}


void loop() {
  // Read from external
  float Xdistance, Ydistance;
  float V_x, V_y;
  long V_total;  // Read from external

  /**************************************************************************************************************************/
  /*   Read from Raspberry

  */
  /**************************************************************************************************************************/


  /**************************************************************************************************************************/
  /*   Sensor commands
  int sensorReading = analogRead(A0);
  if(sensorReading || Ydistance<= 50)     //emergency brake
  {
    disableMotors();
  }
  */
  /**************************************************************************************************************************/


  // linear speed control

  distance_error = Ydistance - distance_setpoint;  // Error in distance
  linearPID.Compute();                             // Compute linear speed (linear_speed)
  if (Ydistance > SAFE_DISTANCE_MAX) {              // Speedup if too far away
    // Too far: Speed up proportionally
    linear_speed = linear_speed + (V_x + (Xdistance - SAFE_DISTANCE_MAX)) * 0.3;  //factor to change
  } 
  else {
  }

  // Limit the target speed to the maximum speed
  if (linear_speed > default_maxSpeedLimit) {
    linear_speed = default_maxSpeedLimit;
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

  // Calculate angular error and compute angular speed
  theta_error = theta_setpoint;
  angularPID.Compute();

  // Convert Linear and Angular Speed to Wheel Speeds
  Stepps_R = (linear_speed + angular_speed * wheelbase / 2.0) / wheel_radius * 60 / (2 * PI);  // Right wheel rotation speed (rad/s)
  Stepps_L = (linear_speed - angular_speed * wheelbase / 2.0) / wheel_radius * 60 / (2 * PI);  // Left wheel rotation speed (rad/s)
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


  // display the updated speed and acceleration to the motors
  // Debugging output (optional, for monitoring the system)
  Serial.print("Distance: ");
  Serial.print(Xdistance);
  Serial.print(",");
  Serial.print(Ydistance);
  Serial.println(" | Car_Speed: ");
  Serial.print(linear_speed);
  Serial.print(" | Turning: ");
  Serial.print(angular_speed);
  Serial.println(" | Acceleration: ");
  Serial.print(L_Acceleration);
  Serial.println(R_Acceleration);


  // Small delay to avoid overwhelming the system (optional)
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

void InitialMoveTest(long trialmove) {
  moveMotors(trialmove, trialmove);
  moveMotors(0, 0);
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