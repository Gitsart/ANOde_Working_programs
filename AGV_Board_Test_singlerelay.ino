int MOTO = 4;

void setup() {
  pinMode(MOTO, OUTPUT);

}

void loop() {
  digitalWrite(MOTO, HIGH);
  delay(3000);
  digitalWrite(MOTO, LOW);
  delay(3000);

}
