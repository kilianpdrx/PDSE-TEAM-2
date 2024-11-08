#include <SoftwareSerial.h>

SoftwareSerial HC12(10, 11); // HC-12 TX Pin, HC-12 RX Pin

String readBuffer = "";
float valueX, valueY;
bool bouton;

// Définition des broches pour les moteurs
const int motor1Pin1 = 4;
const int motor1Pin2 = 5; 
const int motor2Pin1 = 2;
const int motor2Pin2 = 3; 
const int motor3Pin1 = 8;
const int motor3Pin2 = 9; 
const int motor4Pin1 = 6;
const int motor4Pin2 = 7; 

// Vitesse fixe des moteurs (entre 0 et 255)
const int vitesseConstante = 200;

void setup() {
  Serial.begin(9600); // Open serial port to computer
  HC12.begin(9600);   // Open serial port to HC12
  
  // Configuration des broches des moteurs comme sorties
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);
}

void loop() {
  if (HC12.available()) { // Si des données sont disponibles depuis le HC-12
    lireMessage();
    extraireValeurs();
    afficherDonnees();
    // controlerMoteurs(); // Contrôle les moteurs en fonction des valeurs du joystick
    if (!bouton) { // Si le bouton est actif, on arrête tous les moteurs
    avancer();
    return;
  }
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
  if (bouton) { // Si le bouton est actif, on arrête tous les moteurs
    arreterMoteurs();
    return;
  }

  // Contrôler les moteurs pour avancer, reculer, tourner à droite ou à gauche
  if (valueY > 2.5) { // Avancer si la valeur Y est supérieure au milieu
    avancer();
  } else if (valueY < 2.5) { // Reculer si la valeur Y est inférieure au milieu
    reculer();
  } else if (valueX > 2.5) { // Tourner à droite si la valeur X est supérieure au milieu
    tournerDroite();
  } else if (valueX < 2.5) { // Tourner à gauche si la valeur X est inférieure au milieu
    tournerGauche();
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
  digitalWrite(motor3Pin1, LOW);
  digitalWrite(motor3Pin2, LOW);
  digitalWrite(motor4Pin1, LOW);
  digitalWrite(motor4Pin2, LOW);
}

// Fonction pour faire avancer tous les moteurs
void avancer() {
  analogWrite(motor1Pin1, vitesseConstante);
  digitalWrite(motor1Pin2, LOW);
  analogWrite(motor2Pin1, vitesseConstante);
  digitalWrite(motor2Pin2, LOW);
  analogWrite(motor3Pin1, vitesseConstante);
  digitalWrite(motor3Pin2, LOW);
  analogWrite(motor4Pin1, vitesseConstante);
  digitalWrite(motor4Pin2, LOW);
}

// Fonction pour faire reculer tous les moteurs
void reculer() {
  digitalWrite(motor1Pin1, LOW);
  analogWrite(motor1Pin2, vitesseConstante);
  digitalWrite(motor2Pin1, LOW);
  analogWrite(motor2Pin2, vitesseConstante);
  digitalWrite(motor3Pin1, LOW);
  analogWrite(motor3Pin2, vitesseConstante);
  digitalWrite(motor4Pin1, LOW);
  analogWrite(motor4Pin2, vitesseConstante);
}

// Fonction pour tourner à droite
void tournerDroite() {
  analogWrite(motor1Pin1, vitesseConstante);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  analogWrite(motor2Pin2, vitesseConstante);
  analogWrite(motor3Pin1, vitesseConstante);
  digitalWrite(motor3Pin2, LOW);
  digitalWrite(motor4Pin1, LOW);
  analogWrite(motor4Pin2, vitesseConstante);
}

// Fonction pour tourner à gauche
void tournerGauche() {
  digitalWrite(motor1Pin1, LOW);
  analogWrite(motor1Pin2, vitesseConstante);
  analogWrite(motor2Pin1, vitesseConstante);
  digitalWrite(motor2Pin2, LOW);
  digitalWrite(motor3Pin1, LOW);
  analogWrite(motor3Pin2, vitesseConstante);
  analogWrite(motor4Pin1, vitesseConstante);
  digitalWrite(motor4Pin2, LOW);
}
