#include <SoftwareSerial.h>

// Configuration du Bluetooth (SoftwareSerial sur les pins 10 et 11)
SoftwareSerial HC05(5, 3); // HC-05 TX Pin, HC-05 RX Pin


// variables for the app
String readBuffer = "";
float valueX, valueY;
bool bouton;
float delta = 0.2;
const float milieu = 2.5;
int manual = 0;
int stop_all = 0;

// variables for the RP
float prof = -1;       // Profondeur
float mdist = -1;      // Distance X
int tracking = -1;   // Suivi actif

float Xdist = 2, Ydist = 2;  // in m

void setup() {
  Serial.begin(9600);
  HC05.begin(9600);
}

void loop() {

  process_app();
  process_RP();

  // FSM
  if (stop_all == 1){ // KILLSWITCH
    HC05.println("STOP");
  }
  else {
    if (manual == 1){ // MANUAL MODE
      HC05.println("Manual mode");
    }
    else { // AUTO MODE
      if (tracking == -2) { // IF LOST
        HC05.println("LOST, go back to the frame");
      } else if (tracking == 1){ // NORMAL FOLLOWING MODE
        HC05.println("I follow you");
      }
    }
  }


  delay(200); // NECESSARY
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




