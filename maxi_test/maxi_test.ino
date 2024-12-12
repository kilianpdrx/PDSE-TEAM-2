void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Lire une ligne complète
    int separator = data.indexOf(',');         // Trouver la virgule
    if (separator != -1) {
      float distance = data.substring(0, separator).toFloat();
      float angle = data.substring(separator + 1).toFloat();
      
      // Afficher les valeurs reçues
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.print(" m, Angle: ");
      Serial.print(angle);
      Serial.println(" degrés");
    }
  }
}
