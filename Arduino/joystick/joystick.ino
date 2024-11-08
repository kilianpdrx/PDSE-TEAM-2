int axeX = A0; // signal de l'axe X sur entrée A0
int axeY = A1; // signal de l'axe Y sur entrée A1
int BP7 = 7; // bouton-poussoir en broche 7
float X, Y; // valeur des axes X et Y en Volt
bool bouton; // valeur du bouton (poussé ou non)

void setup (){
  pinMode (axeX, INPUT); // définition de A0 comme une entrée
  pinMode (axeY, INPUT); // définition de A1 comme une entrée
  pinMode (BP7, INPUT); // définition de la broche 7 comme une entrée
  digitalWrite(BP7, HIGH); // Activation de la résistance de Pull-Up
  Serial.begin (9600);
}

void loop (){
  X = analogRead (axeX) * (5.0 / 1023.0);
  Y = analogRead (axeY) * (5.0 / 1023.0);
  bouton = digitalRead (BP7);
  Serial.print ("Axe X:");
  Serial.print (X, 4);
  Serial.print ("V, ");
  Serial.print ("Axe Y:");
  Serial.print (Y, 4);
  Serial.print ("V, ");
  Serial.print ("Bouton:");

  if (bouton==1){
    Serial.println (" Aucune pression sur le bouton poussoir ");
  }
  else{
    Serial.println (" Bouton-poussoir actif ");
  }
  
  delay (500);
}