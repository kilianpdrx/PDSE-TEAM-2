void setup() {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Lire une ligne complète
    
    int firstSeparator = data.indexOf(',');    // Trouver la première virgule
    int secondSeparator = data.indexOf(',', firstSeparator + 1); // Trouver la deuxième virgule

    if (firstSeparator != -1 && secondSeparator != -1) {
      float prof = data.substring(0, firstSeparator).toFloat(); // Extraire la première valeur
      float mdist = data.substring(firstSeparator + 1, secondSeparator).toFloat(); // Extraire la deuxième valeur
      float tracking = data.substring(secondSeparator + 1).toFloat(); // Extraire la troisième valeur
      
      // Afficher les valeurs reçues
      Serial.print("Profondeur: ");
      Serial.print(prof);
      Serial.print(" m, X_dist: ");
      Serial.print(mdist);
      Serial.print(" m, Tracking: ");
      Serial.println(tracking);
    }
  }
}
