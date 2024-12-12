int intData = 0;

int angle = 0;
int dist = 0;

String data = "";


void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(8, OUTPUT);
}


void loop() {
  if (Serial.available() > 0) {
    data = Serial.readStringUntil('\n');
    angle = data.toInt();

    data = Serial.readStringUntil('\n');
    dist = data.toInt();


    if (dist != 0){
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);
    }

    if (angle != 0){
      digitalWrite(8, HIGH);
    }
    else{
      digitalWrite(8, LOW);
    }
  }
}