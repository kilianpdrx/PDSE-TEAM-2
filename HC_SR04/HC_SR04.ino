const int trig1 = 4;
const int echo1 = 5;

const int trig2 = 6;
const int echo2 = 7;

bool light = false;

float distance1, duration1, distance2, duration2;

void setup() {
  // put your setup code here, to run once:
  pinMode(trig1, OUTPUT);  
	pinMode(echo1, INPUT);  
  pinMode(trig2, OUTPUT);  
	pinMode(echo2, INPUT);  
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN, light);

  digitalWrite(trig1, LOW);
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  delayMicroseconds(10);
  duration1 = pulseIn(echo1, HIGH);

  digitalWrite(trig2, LOW);
  delayMicroseconds(2);
  digitalWrite(trig2, HIGH);
  delayMicroseconds(10);
  duration2 = pulseIn(echo2, HIGH);

  distance1 = 0.0343*duration1*0.5;
  distance2 = 0.0343*duration2*0.5;

  Serial.println(distance1);
  Serial.println(distance2);

  if(distance1 < 20 || distance2 < 20){
    light = true;
  }
  else{
    light = false;
  }

  delay(100);

}
