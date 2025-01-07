#include <SoftwareSerial.h>
#include <AccelStepper.h>

// Configuration du Bluetooth (SoftwareSerial sur les pins 10 et 11)
SoftwareSerial HC05(13, 9); // HC-05 TX Pin, HC-05 RX Pin


// variables for the app
String readBuffer = "";
float valueX, valueY;
bool bouton;
float delta = 20;
const float milieu = 100;
int manual = 0;
int stop_all = 0;

// variables for the RP
float prof = -1;       // Profondeur
float mdist = -1;      // Distance X
int tracking = -1;   // Suivi actif

float Xdist = 2, Ydist = 2;  // in m

//variables for speed calculations
const float vitesseMax = 0.5;
const double wheel_diameter = 0.14;
float speedfactor = 305 / 300;
double vitesseGauche_step, vitesseDroite_step, vitesse, theta, vitesseDroite, vitesseGauche, r, offsetY, offsetX;

// Driver Connection Pins (constant current, step/direction bipolar motor driver)
const int dirPin_left = 11, stepPin_left = 12;
const int ena_left = 10;
const int dirPin_right = 5, stepPin_right = 4;
const int ena_right = 6;
// Creates an instance - works for (Bipolar, constant current, step/direction driver)
AccelStepper leftwheel(AccelStepper::DRIVER, stepPin_left, dirPin_left);
AccelStepper rightwheel(AccelStepper::DRIVER, stepPin_right, dirPin_right);  //current position set as 0, enable signal

void setup() {
  rightwheel.setPinsInverted(true, false, false);
  leftwheel.setPinsInverted(false, false, false);
  Serial.begin(9600);
  HC05.begin(9600);
  //leftwheel.setAcceleration(300);
  //rightwheel.setAcceleration(300);
  leftwheel.setMaxSpeed(0);
  rightwheel.setMaxSpeed(0);
}

void loop() {

  process_app();
  //manual = 1;
  //process_RP();
  // FSM
  if (stop_all == 1){ // KILLSWITCH
    HC05.println("STOP");
    setSpeedAcceleration(0, 0, 0, 0);
    disableMotors();
    /*for (int i=0;i<1000;i++){
        moveMotors(10000, 10000);
        rightwheel.run();
        leftwheel.run();
      }*/
  }
  else {
    if (manual == 1){ // MANUAL MODE
      HC05.println("Manual mode");
      //HC05.print(valueX);
      controlerMoteurs(valueX, valueY);
      //setSpeedAcceleration(1000,100, 1000, 100);
      //Serial.println("finish!");
      for (int i=0;i<1000;i++){
        moveMotors(100000, 100000);
        rightwheel.run();
        leftwheel.run();
      }
    }
    else { // AUTO MODE
      if (tracking == -2) { // IF LOST
        HC05.println("LOST, go back to the frame");
        setSpeedAcceleration(0, 0, 0, 0);
        disableMotors();
      } else if (tracking == 1){ // NORMAL FOLLOWING MODE
        HC05.println("I follow you");
        setSpeedAcceleration(0, 0, 0, 0);
        disableMotors();
      }
    }
  }


  //delay(0.1); // NECESSARY
}


// Fonction pour extraire les valeurs de X, Y et Bouton du message
void process_app() {
  if (HC05.available()) {
    readBuffer = HC05.readStringUntil('e'); // Lire jusqu'au délimiteur de fin 'e'
    int indexX = readBuffer.indexOf("X:");
    int indexY = readBuffer.indexOf("Y:");
    int indexB = readBuffer.indexOf("B:");

    int indexSTOP = readBuffer.indexOf("STOP:");
    int indexMANUAL = readBuffer.indexOf("MANUAL:");

    if (indexX != -1 && indexY != -1 && indexB != -1) { // Si toutes les valeurs sont présentes
      valueX = readBuffer.substring(indexX + 2, indexY - 1).toFloat();
      valueY = readBuffer.substring(indexY + 2, indexB - 1).toFloat();
      bouton = readBuffer.substring(indexB + 2, indexSTOP - 1).toInt(); // Convertir en entier pour obtenir 0 ou 1
      stop_all = readBuffer.substring(indexSTOP + 5, indexMANUAL - 1).toInt();
      manual = readBuffer.substring(indexMANUAL + 7).toInt();
    }
  }

}

// Fonction pour afficher les valeurs reçues pour le débogage
void afficherDonnees_app() {
  Serial.print("Axe X: "); Serial.print(valueX); Serial.print("V, ");
  Serial.print("Axe Y: "); Serial.print(valueY); Serial.print("V, ");
  Serial.print("Bouton: "); Serial.println(bouton ? "Eteint" : "Actif");
  Serial.print("Manual: "); Serial.println(manual ? "Manual" : "Auto");
  Serial.print("KILL: "); Serial.println(stop_all ? "ON" : "OFF");
}


void process_RP(){
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
}

void controlerMoteurs(float valueX, float valueY) {
  enableMotors();

  /*if (!bouton) { // Arrêter les moteurs si bouton pressé
    arreterMoteurs();
    return;
  }*/
  offsetX = valueX - milieu;
  offsetY = -valueY + milieu;
  //Serial.println(offsetX);
  //Serial.println(offsetY);
  constrain(offsetX, -100, 100);
  constrain(offsetY, -100, 100);
  r = sqrt(offsetX * offsetX + offsetY * offsetY);
  theta = atan2(offsetX, offsetY);

  if(theta <PI/4 && theta > 0 ){
    theta = PI/8;    
  }
  else if (theta < PI/2 && theta > PI/4){
    theta = 3*PI/8;
  }else if (theta <0 && theta > -PI/4){
    theta = -PI/8;
  }else if (theta < -PI/4 && theta > -PI/2){
    theta = -3*PI/8;
  }else if (theta < -PI/2 && theta > -3*PI/4){
    theta = -5*PI/8;
  }else if (theta < -3*PI/4 && theta > -PI){
    theta = -7*PI/8;
  }else if (theta < 3*PI/4 && theta > PI/2){
    theta = 5*PI/8;
  }else if (theta < PI && theta > 3*PI/4){
    theta = 7*PI/8;
  }
  
  
  if (r < delta) {
    arreterMoteurs();
    return;
  }
  else{
    if(offsetY < 0)
    {
      rightwheel.setPinsInverted(false, false, false);
      leftwheel.setPinsInverted(true, false, false);
    }
    else
    {
      rightwheel.setPinsInverted(true, false, false);
      leftwheel.setPinsInverted(false, false, false);
    }
    vitesse = 0.7;//r/ 145 * vitesseMax;
  //float vitesse = map(r, 0, 145, 0, vitesseMax);
    //Serial.print(vitesse);
    vitesseGauche = vitesse * (1 + 2 * sin(theta)/2 );
    vitesseDroite = vitesse * (1 - 2 * sin(theta)/2 );
    vitesseGauche_step = vitesseGauche*400/PI/wheel_diameter;
    vitesseDroite_step = vitesseDroite*400/PI/wheel_diameter;
    vitesseDroite_step = vitesseDroite_step * speedfactor;
  }
  
  /*
  Serial.println("theta");
  Serial.println(theta);
  Serial.println("VG_speed:");
  Serial.println(vitesseGauche_step);
  Serial.println("VD_speed:");
  Serial.println(vitesseDroite_step);
  */

  setSpeedAcceleration(vitesseGauche_step, 100, vitesseDroite_step, 100);
}

void enableMotors() {
    leftwheel.enableOutputs();
    rightwheel.enableOutputs();
    digitalWrite(ena_left, LOW);
    digitalWrite(ena_right, LOW);
}

void arreterMoteurs() {
  setSpeedAcceleration(0, 0, 0, 0);
  disableMotors();
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