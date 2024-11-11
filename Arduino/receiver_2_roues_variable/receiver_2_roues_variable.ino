#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

String readBuffer = "";
float valueX, valueY;
bool bouton;

float delta = 0.2;

// Définition des broches pour les moteurs avant
const int motor1Pin1 = 6; // Avant gauche
const int motor1Pin2 = 7;
const int motor2Pin1 = 8; // Avant droit
const int motor2Pin2 = 9;

// Définition des limites pour le calcul de la vitesse
const int vitesseMax = 255;
const float milieu = 2.5;

void setup() {
  Serial.begin(9600); // Open serial port to computer
  HC12.begin(9600);   // Open serial port to HC12
  
  // Configuration des broches des moteurs comme sorties
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
}

void loop() {
  if (HC12.available()) { // Si des données sont disponibles depuis le HC-12
    lireMessage();
    extraireValeurs();
    afficherDonnees();
    controlerMoteurs(); // Contrôle les moteurs en fonction des valeurs du joystick
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

// Fonction pour contrôler les moteurs en fonction de la position du joystick
void controlerMoteurs() {
  if (!bouton) { // Si le bouton est actif, on arrête tous les moteurs
    arreterMoteurs();
    return;
  }

  // Calcul des vitesses en fonction de l'axe X et Y
  int vitesseY = map(abs(valueY - milieu), 0, 2.5, 0, vitesseMax);
  int vitesseX = map(abs(valueX - milieu), 0, 2.5, 0, vitesseMax);

  // Contrôler les moteurs pour avancer, reculer, tourner à droite ou à gauche
  if (valueY > milieu + delta) { // Avancer si la valeur Y est supérieure au milieu
    avancer(vitesseY);
  } else if (valueY < milieu - delta) { // Reculer si la valeur Y est inférieure au milieu
    reculer(vitesseY);
  } else if (valueX > milieu + delta) { // Tourner à droite si la valeur X est supérieure au milieu
    tournerDroite(vitesseX);
  } else if (valueX < milieu - delta) { // Tourner à gauche si la valeur X est inférieure au milieu
    tournerGauche(vitesseX);
  } else { // Si aucune condition n'est remplie, arrêter les moteurs
    arreterMoteurs();
  }
}

// Fonction pour arrêter tous les moteurs
void arreterMoteurs() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

// Fonction pour faire avancer les moteurs avant avec une vitesse variable
void avancer(int vitesse) {
  analogWrite(motor1Pin1, vitesse);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(motor2Pin1, vitesse);
  digitalWrite(motor2Pin2, LOW);
}

// Fonction pour faire reculer les moteurs avant avec une vitesse variable
void reculer(int vitesse) {
  digitalWrite(motor1Pin1, LOW);
  analogWrite(motor1Pin2, vitesse);
  digitalWrite(motor2Pin1, LOW);
  analogWrite(motor2Pin2, vitesse);
}

// Fonction pour tourner à droite avec une vitesse variable
void tournerDroite(int vitesse) {
  analogWrite(motor1Pin1, vitesse);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  analogWrite(motor2Pin2, vitesse);
}

// Fonction pour tourner à gauche avec une vitesse variable
void tournerGauche(int vitesse) {
  digitalWrite(motor1Pin1, LOW);
  analogWrite(motor1Pin2, vitesse);
  analogWrite(motor2Pin1, vitesse);
  digitalWrite(motor2Pin2, LOW);
}
