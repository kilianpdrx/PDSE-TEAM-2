#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

String readBuffer = "";
float valueX, valueY;
bool bouton;

//Motor 1
const int motorPin1 = 4;
const int motorPin2 = 5; 

//Motor 2
const int motorPin1 = 2;
const int motorPin2 = 3; 

//Motor 3
const int motorPin1 = 8;
const int motorPin2 = ; 

//Motor 4
const int motorPin1 = 9;
const int motorPin2 = 8; 





void setup() {
  Serial.begin(9600); // Open serial port to computer
  HC12.begin(9600);   // Open serial port to HC12
}

void loop() {
  if (HC12.available()) { // Si des données sont disponibles depuis le HC-12
    lireMessage();
    extraireValeurs();
    afficherDonnees();
  }
  delay(200); // Intervalle de traitement
}

// Fonction pour lire le message reçu via HC-12
void lireMessage() {
  readBuffer = HC12.readStringUntil('e'); // Lire jusqu'au délimiteur de fin 'e'
}

// Fonction pour extraire les valeurs de X, Y et Bouton du message
void extraireValeurs() {
  int indexX = readBuffer.indexOf("X:");
  int indexY = readBuffer.indexOf("Y:");
  int indexB = readBuffer.indexOf("B:");

  if (indexX != -1 && indexY != -1 && indexB != -1) { // Si toutes les valeurs sont présentes
    valueX = readBuffer.substring(indexX + 2, indexY - 1).toFloat();
    valueY = readBuffer.substring(indexY + 2, indexB - 1).toFloat();
    bouton = readBuffer.substring(indexB + 2).toInt(); // Convertir en entier pour obtenir 0 ou 1
  }
}

// Fonction pour afficher les valeurs reçues pour le débogage
void afficherDonnees() {
  Serial.print("Axe X: "); Serial.print(valueX); Serial.print("V, ");
  Serial.print("Axe Y: "); Serial.print(valueY); Serial.print("V, ");
  Serial.print("Bouton: "); Serial.println(bouton ? "Eteint" : "Actif");
}
