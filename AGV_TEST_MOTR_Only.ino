int motor1_REV = 4;
int motor1_fwd = 5;
int mot1_brake = 6;
int motor2_REV = 9;
int motor2_fwd = 11;
int mot2_brake = 12;
void setup() {
  pinMode(motor1_REV, OUTPUT);
  pinMode(motor2_REV, OUTPUT);
  pinMode(motor1_fwd, OUTPUT);
  pinMode(mot1_brake, OUTPUT);
  pinMode(mot2_brake, OUTPUT);
  pinMode(motor2_fwd, OUTPUT);

}


void reverse(){
   //analogWrite(motor1_speed,20);
   // analogWrite(motor2_speed,20);
  digitalWrite(mot1_brake,LOW);
  digitalWrite(mot2_brake,LOW);
  digitalWrite(motor1_REV,HIGH);                                    
  digitalWrite(motor2_fwd,HIGH);
  digitalWrite(motor2_REV,LOW);
  digitalWrite(motor1_fwd,LOW);  
}

void forward(){
  digitalWrite(mot1_brake,LOW);
  digitalWrite(mot2_brake,LOW);
  digitalWrite(motor1_REV,LOW);
  digitalWrite(motor2_fwd,LOW);
  digitalWrite(motor1_fwd,HIGH); //(LOW = 5V , HIGH = 0V )
  digitalWrite(motor2_REV,HIGH); 
}
void brake(){
  digitalWrite(mot1_brake,HIGH);
  digitalWrite(mot2_brake,HIGH);
  digitalWrite(motor1_fwd,LOW); //(LOW = 5V , HIGH = 0V )
  digitalWrite(motor1_REV,LOW); 
  digitalWrite(motor2_fwd,LOW);
  digitalWrite(motor2_REV,LOW);
}

void loop() {
  forward();
  delay(6000);
  
  reverse();
  delay(6000);

}
