int laser = 8;

void setup ()

{

pinMode (laser, OUTPUT);

}

void loop () {

  digitalWrite (laser, HIGH);

  delay (10);

  digitalWrite (laser, LOW);

  delay (100);

}