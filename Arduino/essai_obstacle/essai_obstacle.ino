int count;

void setup () {

  Serial.begin (9600);

  pinMode (9, INPUT); // Sélection de la broche de sortie

}

void loop (){

  Serial.print ("Sensor: ");

  Serial.println (digitalRead(9)); // Affiche l'état de la sortie du capteur

  delay (500);

}