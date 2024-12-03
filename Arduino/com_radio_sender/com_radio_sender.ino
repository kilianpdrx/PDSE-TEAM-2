#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

// JOYSTICK
int axeX = A0; // signal de l'axe X sur entrée A0
int axeY = A1; // signal de l'axe Y sur entrée A1
int BP7 = 7;   // bouton-poussoir en broche 7
float X, Y;    // valeur des axes X et Y en Volt
bool bouton;   // valeur du bouton (poussé ou non)



void setup() {
  HC12.begin(9600);               // Open serial port to HC12
  Serial.begin(9600);
  delay(100);

  pinMode(axeX, INPUT); // définition de A0 comme une entrée
  pinMode(axeY, INPUT); // définition de A1 comme une entrée
  pinMode(BP7, INPUT);  // définition de la broche 7 comme une entrée
  digitalWrite(BP7, HIGH); // Activation de la résistance de Pull-Up
}

void loop() {
  lireJoystick();
  envoyerDonnees();
  afficherDonnees();
  delay(200);  // Intervalle de transmission
}

// Fonction pour lire les valeurs des axes et du bouton
void lireJoystick() {
  X = analogRead(axeX) * (5.0 / 1023.0);
  Y = analogRead(axeY) * (5.0 / 1023.0);
  bouton = digitalRead(BP7);
}

// Fonction pour envoyer les données via HC-12
void envoyerDonnees() {
  HC12.print("sX:" + String(X, 2) + ";Y:" + String(Y, 2) + ";B:" + String(bouton) + "e");
}

// Fonction pour afficher les valeurs lues pour le débogage
void afficherDonnees() {
  Serial.print("Axe X: "); Serial.print(X, 2); Serial.print("V, ");
  Serial.print("Axe Y: "); Serial.print(Y, 2); Serial.print("V, ");
  Serial.print("Bouton: "); Serial.println(bouton ? "Eteint" : "Actif");
}
