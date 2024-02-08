///signature 1 and 2 for speed              ---- green
// signature 3 is slow                      ---- not used
// signature 4 is for pause ,operator acknowledge, continue straight --- not used
// signature 5 is for  pause operator acknowlege,rotate              --- red
// signature 6 is to rotate                                          --- yellow

//com port 18




#include <TimerOne.h>
#include <Pixy2.h>
Pixy2 pixy;

//input outputs//

int red = A3;
int green = A4;
int buzzer = A5;
int Start_pb = 3;  //was2
int Emergency = A1;  //was3
int mot1_reverse = 4;
int mot1_forward = 5;
int mot1_brake = 6;
int mot1_speed = 7;
int mot2_reverse = 9;
int mot2_forward = 11;
int mot2_brake = A15; // was12
int mot2_speed =8;
int voltage_input = A0;
int scanner = 10;
int u_turn = 2;
//input outputs//


//PID CONTROL//
//float Kp=0.0007;
//float Ki=0.0000008;
//float Kd=0.0000075;
float Kp=0.0027;
float Ki=0.0000013;
float Kd=0.08;


int P;
int I;
int D;
int lastError = 0;

const uint8_t basespeeda = 35;
const uint8_t basespeedb = 35;



//Variables//
bool u_turning = false;
bool lidar_out = false;
bool off_path = false;
bool battery_low = false;
bool pause = false;
bool op_ack = false;
bool obstruction = LOW;
bool Start_process = false;
bool Stop_process = false;

int i, j, l, k = 0;
int distance, distance_1, distance_2 = 0;
int signature = 0;
int x = 0;                      //positon x axis
int y = 0;                      //position y axis
unsigned int width_1 = 0;         //object's width_1
unsigned int height_1 = 0;        //object's height_1
int map_1, map_2 = 0;
bool rotate = false;
int move_slow = 0;
int count = 0;
int cameraerror = 0;
int rotate_count = 0;
int val = 0;
int volt = 0;

void writeString(String stringData)
{ // Used to serially push out a String with Serial.write()

  for (int i = 0; i < stringData.length(); i++)
  {
    Serial1.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
  }

  Serial1.write(0xff); //We need to write the 3 ending bits to the Nextion as well
  Serial1.write(0xff); //it will tell the Nextion that this is the end of what we want to send.
  Serial1.write(0xff);
}


void camera()
{
  uint16_t blocks;
  blocks = pixy.ccc.getBlocks();//receive data from pixy
  signature = pixy.ccc.blocks[i].m_signature;    //get object's signature
  x = pixy.ccc.blocks[i].m_x;                    //get x position
  y = pixy.ccc.blocks[i].m_y;                    //get y position
  width_1 = pixy.ccc.blocks[i].m_width;            //get width_1
  height_1 = pixy.ccc.blocks[i].m_height;          //get height_1
  int i;

  //  Serial.print("Detected ");
  if (pixy.ccc.numBlocks)
  {
    cameraerror = 0;
//    Serial.print("Detected ");
//    Serial.println(pixy.ccc.numBlocks);
//        for (i = 0; i < pixy.ccc.numBlocks; i++)
//        {
//          pixy.ccc.blocks[i].print();
//        }
  }
  else
  {
    cameraerror = 1;
    //    Serial.print("not_Detected ");
    //    Serial.println(last_value[1]);
  }
  if ((signature != 1) && (signature != 2) && (signature != 3) && (signature != 4) && (signature != 5) && (signature != 6) && (rotate == false))
  {

    digitalWrite(mot1_brake, HIGH);
    digitalWrite(mot2_brake, HIGH);
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n8.val="; //Build the part of the string that we know
    sendThis.concat(1); //Add the variable we want to send
    writeString(sendThis);
  }
  else if (((signature == 1) || (signature == 2) || (signature == 3)) || (signature == 4) || (signature == 5) || (signature == 6) && cameraerror == 1)
  {

    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n8.val="; //Build the part of the string that we know
    sendThis.concat(0); //Add the variable we want to send
    writeString(sendThis);
    cameraerror = 0;
  }
  if (lidar_out == true)
  {
    move_slow = 1;
    count = 0;
  }
  if(move_slow == 1 || signature == 3)
   {
    val = 10;
    map_2 = map(x, 157, 314, (val), 5);
    map_1 = map(x, 157, 314, (val), 15);
   }
  
}

void sensor()
{
  if (obstruction == false)
  {
    lidar_out = true;

    digitalWrite(mot1_brake, HIGH);
    digitalWrite(mot2_brake, HIGH);

    move_slow = 1;
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n4.val="; //Build the part of the string that we know
    sendThis.concat(1); //Add the variable we want to send
    writeString(sendThis);
    obstruction = digitalRead(scanner);
    digitalWrite(red, HIGH);
    digitalWrite(buzzer, HIGH);
    digitalWrite(green, LOW);
    sendThis = "n4.val="; //Build the part of the string that we know
    sendThis.concat(0); //Add the variable we want to send
    writeString(sendThis);
    move_slow = 1;
    count = 0;
  }
  else
  {
    obstruction = digitalRead(scanner);
    if ((obstruction == true ) && (lidar_out == true))
    {
      move_slow = 1;
      count = 0;
      lidar_out = false;
      delay(100);
      digitalWrite(red, LOW);
    digitalWrite(buzzer, LOW);
    digitalWrite(green, HIGH);
     // digitalWrite(motor1_dir, HIGH);
     // digitalWrite(motor2_dir, HIGH);
      digitalWrite(mot1_brake, LOW);
      digitalWrite(mot2_brake, LOW);
      String sendThis = ""; //Declare and initialise the string we will send
      sendThis = "n4.val="; //Build the part of the string that we know
      sendThis.concat(0); //Add the variable we want to send
      writeString(sendThis);
    }
  }
}

void right_rotation()
{
  if (rotate_count < 200)
  {
    //digitalWrite(motor1_dir, LOW);
   // digitalWrite(motor2_dir, LOW);
    rotate_count ++;
  }
  else if (rotate_count >= 200 )
  {
    analogWrite(mot2_speed, 3);
    analogWrite(mot1_speed, 3);
    digitalWrite(mot1_brake, LOW);
    digitalWrite(mot2_brake, LOW);
    digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH); 
    digitalWrite(mot2_forward,LOW);
    digitalWrite(mot2_reverse,HIGH);
 
    rotate_count ++;
  }
}

void left_rotation()
{
  if (rotate_count < 200)
  {
   // digitalWrite(motor1_dir, LOW);
    //digitalWrite(motor2_dir, LOW);
    rotate_count ++;
  }
  else if (rotate_count >= 300 )
  {
    analogWrite(mot2_speed, 3);
    analogWrite(mot1_speed, 3);
  digitalWrite(mot1_brake, LOW);
  digitalWrite(mot2_brake,LOW);
  digitalWrite(mot1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
  digitalWrite(mot1_reverse,LOW); 
  digitalWrite(mot2_forward,HIGH);
  digitalWrite(mot2_reverse,LOW); 
    rotate_count ++;
  }
}



void push_button()
{
  if  ((digitalRead(Emergency) == HIGH) && (digitalRead(Start_pb) == LOW)  && battery_low == false && Start_process == false)
  {

    move_slow = 1;
    count = 0;
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n1.val="; //Build the part of the string that we know
    sendThis.concat(1); //Add the variable we want to send
    writeString(sendThis);
    move_slow = 1;
    op_ack = false;
    sendThis = "n2.val="; //Build the part of the string that we know
    sendThis.concat(0); //Add the variable we want to send
    writeString(sendThis);
    delay(100);
    Start_process = true;
    Stop_process = false;
    sendThis = "n1.val="; //Build the part of the string that we know
    sendThis.concat(1); //Add the variable we want to send
    writeString(sendThis);
    move_slow = 1;
    sendThis = "n3.val="; //Build the part of the string that we know
    sendThis.concat(volt); //Add the variable we want to send
    writeString(sendThis);
  digitalWrite(mot1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
  digitalWrite(mot1_reverse,LOW); 
  digitalWrite(mot2_forward,LOW);
  digitalWrite(mot2_reverse,HIGH);
  Serial.println("FWDPB"); 
    
  }
  else if  ((digitalRead(Emergency) == HIGH) && (digitalRead(Start_pb) == HIGH) && (digitalRead(u_turn) == LOW) && battery_low == false && Start_process == false)
  {
    rotate = true;
    u_turning = true;
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n2.val="; //Build the part of the string that we know
    sendThis.concat(0); //Add the variable we want to send
    writeString(sendThis);
  }

  else if  ((digitalRead(Emergency) == HIGH) && (digitalRead(Start_pb) == LOW) && (pause == false))
  {
    Stop_process = false;
    Start_process = true;
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n2.val="; //Build the part of the string that we know
    sendThis.concat(0); //Add the variable we want to send
    writeString(sendThis);
    Serial.println("MOTION");
  }
  else if  ((digitalRead(Emergency) == HIGH) && (digitalRead(Start_pb) == LOW) && (pause == true))
  {
    op_ack = true;
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n2.val="; //Build the part of the string that we know
    sendThis.concat(0); //Add the variable we want to send
    writeString(sendThis);
    Serial.println("OPACK");
  }
  else if (digitalRead(Emergency) == LOW)// EMG PRESSED
  {
    count = 0;
    move_slow = 1;
    op_ack = false;
    Stop_process = true;
    Start_process = false;
    u_turning = false;
   digitalWrite(mot1_brake, HIGH);
  digitalWrite(mot2_brake, HIGH);
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n2.val="; //Build the part of the string that we know
    sendThis.concat(1); //Add the variable we want to send
    writeString(sendThis);
    delay(100);
    digitalWrite(red, HIGH);
    digitalWrite(buzzer, HIGH);
    digitalWrite(green, LOW);
    Serial.println("EMGPRESS");
  }
}


void motion()
{
 Serial.print("motion");
   if(signature == 1&& (obstruction  == true) && (rotate == false) && (Stop_process == false) && lidar_out == false && move_slow == 0)
 {
      int error = 157- x;
      P = error;
      I = I+error;
      D = error- lastError;
      lastError = error;
//    int motorspeed =(P*Kp +I*Ki+D*Kd)*basespeeda;
      int motorspeed=((P*Kp)+(I*Ki)+(D*Kd))*basespeeda;
    
      int motorspeeda = basespeeda + motorspeed;
      int motorspeedb = basespeedb - motorspeed;
      cameraerror = 0;
      rotate_count = 0;
      op_ack = false;
       pause = false;
       
     digitalWrite(mot1_brake, LOW);
    digitalWrite(mot2_brake,LOW);
    digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW); 
      analogWrite(mot1_speed, motorspeedb);
      analogWrite(mot2_speed, motorspeeda);
      Serial.println("motor speed: "+String(motorspeed)+" error: "+ error +" m1:"+ String(motorspeeda) +" m2:"+String(motorspeedb));
  }
 else if(signature == 1&& (obstruction  == true) && (rotate == false) && (Stop_process == false) && lidar_out == false && move_slow == 1)
  {
     if (0 <= x && x <= 157)
  {
    cameraerror = 0;
    rotate_count = 0;
    op_ack = false;
    pause = false;
    move_slow = 1;
    

    analogWrite(mot1_speed, map_1);
    analogWrite(mot2_speed, map_2);
     digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);                                    
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW); 
    digitalWrite(mot1_brake, LOW);
    digitalWrite(mot2_brake, LOW);
    
    move_slow = 1;
    count++;
  }
   else if (( 157 < x && x <= 314))
  {
    cameraerror = 0;
    rotate_count = 0;
    op_ack = false;
    pause = false;
    move_slow = 1; 
   
    analogWrite(mot1_speed, map_1);
    analogWrite(mot2_speed, map_2);
    digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);                                    
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW); 
    digitalWrite(mot1_brake, LOW);
    digitalWrite(mot2_brake, LOW);
    
    move_slow = 1;
    count++;
  }
     if (count >= 1000)
    {
      move_slow = 0;
      count = 0;
    }
}



  else if ((signature == 3) && (0 <= x && x <= 157) && (obstruction  == true) && (rotate == false) && (Stop_process == false) && lidar_out == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    op_ack = false;
    pause = false;
    move_slow = 1;
   
  
    analogWrite(mot1_speed, map_1);
    analogWrite(mot2_speed, map_2);
    digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);                                    
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW); 
    digitalWrite(mot1_brake, LOW);
    digitalWrite(mot2_brake, LOW);
    Serial.println("x: "+String(x)+" mot1_speed:"+String(map_1)+" mot2:"+String(map_2));
  }

  else if ((signature == 3) && ( 157 < x && x <= 314) && (obstruction  == true) && (rotate == false) && (Stop_process == false) && lidar_out == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    op_ack = false;
    pause = false;
    move_slow = 1;
    
 
    analogWrite(mot1_speed, map_1);
    analogWrite(mot2_speed, map_2);
    digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);                                    
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW); 
    digitalWrite(mot1_brake, LOW);
    digitalWrite(mot2_brake, LOW);
    
    move_slow = 1;
  }
 
  
  
  else if ((signature == 5 && width_1 > 20 && obstruction  == true && pause == false) || (signature == 5 && signature == 3  && obstruction  == true && pause == false) && lidar_out == false)
  {
    delay(10);
    pause = true;
    digitalWrite(mot1_brake, HIGH);
    digitalWrite(mot2_brake, HIGH);
    delay(10);
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n6.val="; //Build the part of the string that we know
    sendThis.concat(1);
    writeString(sendThis);
  }
  else if (((signature == 5) && (width_1 > 20) && (obstruction  == true) && pause == true && op_ack == false) && lidar_out == false)
  {
    Serial.println("enterrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrredddddddddddddddddddddddddddd 5");
    digitalWrite(mot1_brake, HIGH);
    digitalWrite(mot2_brake, HIGH);
    digitalWrite(red, HIGH);
    digitalWrite(buzzer, HIGH);
    digitalWrite(green, LOW);
  }
  else if ((signature == 5) && (width_1 > 20) && (obstruction  == true) && pause == true && op_ack == true && lidar_out == false)
  {
    cameraerror = 0;
    move_slow = 1;
    rotate = true;
    right_rotation();
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n6.val="; //Build the part of the string that we know
    sendThis.concat(0);
    writeString(sendThis);
    move_slow = 1;
    //    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n2.val="; //Build the part of the string that we know
    sendThis.concat(0); //Add the variable we want to send
    writeString(sendThis);
  }
  else if ((signature == 6) && (157 < x && x < 314) && (Stop_process == false) && (obstruction  == true) && pause == false && op_ack == false && lidar_out == false)
  {
    val = 5;
    cameraerror = 0;
    move_slow = 1;
    map_1 = map(x, 157, 314, (val), 3);
    map_2 = map(x, 157, 314, (val), 7);
    analogWrite(mot1_speed, map_1);
    analogWrite(mot2_speed, map_2);
    digitalWrite(mot1_brake, LOW);
    digitalWrite(mot2_brake, LOW);
    digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);                                    
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW);
 
    move_slow = 1;
    rotate = true;
   right_rotation();
    move_slow = 1;
  }
  else if ((signature == 6) && (0 <= x && x <= 157) && (Stop_process == false) && (obstruction  == true) && pause == false && op_ack == false && lidar_out == false)
  {
    val = 5;
    cameraerror = 0;
    move_slow = 1;
    map_1 = map(x, 157, 314, (val), 7);
    map_2 = map(x, 157, 314, (val), 3);
    analogWrite(mot1_speed, map_1);
    analogWrite(mot2_speed, map_2);
     digitalWrite(mot1_brake, LOW);
    digitalWrite(mot2_brake, LOW);
    digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);                                    
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW);
 
    move_slow = 1;
    right_rotation();
    rotate = true;
   
  }

  else if ((signature == 1 && rotate == true && width_1 > 70) || (signature == 2 && rotate == true && width_1 > 30) || (signature == 3 && rotate == true && width_1 > 30))
  {
    rotate = false;
    digitalWrite(mot1_brake, HIGH);
    digitalWrite(mot2_brake, HIGH);
    digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);                                    
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW);
    delay(1000);
     digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);                                    
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW);
    move_slow = 1;

  }

  else if ((signature != 1) && (signature != 2) && (signature != 3) && (signature != 4) && (signature != 5) && (signature != 6) && rotate == false)
  {

    digitalWrite(mot1_brake, HIGH);
    digitalWrite(mot2_brake, HIGH);
  }
}

void setup()
{
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  Serial1.begin(9600);// DISPLAY
  pixy.init();
  Timer1.initialize(30000);
  Timer1.attachInterrupt(camera);
  pinMode(mot1_forward, OUTPUT);
  pinMode(mot1_brake, OUTPUT);
  pinMode(mot2_brake, OUTPUT);
  pinMode(mot2_forward, OUTPUT);
  pinMode(mot1_reverse, OUTPUT);
  pinMode(mot2_reverse, OUTPUT);
  pinMode(mot1_speed, OUTPUT);
  pinMode(mot2_speed, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(Start_pb, INPUT_PULLUP);
  pinMode(Emergency, INPUT_PULLUP);
  pinMode(u_turn, INPUT_PULLUP);
  pinMode(voltage_input, INPUT);
  pinMode(scanner, INPUT);
  digitalWrite(red, LOW);
  digitalWrite(buzzer, LOW);
  digitalWrite(green, LOW);

  volt = analogRead(voltage_input);
  String sendThis = ""; //Declare and initialise the string we will send
  sendThis = "n3.val="; //Build the part of the string that we know
  sendThis.concat(volt); //Add the variable we want to send
  writeString(sendThis);
  delay(1000);

}

void loop()
{  
  volt = analogRead(voltage_input);
  obstruction = digitalRead(scanner);
  Serial.println(String(Start_process) + String(digitalRead(Emergency)) + String(u_turning) + String(digitalRead(u_turn)) + String(pause)+ String(lidar_out));
  Serial.println(rotate_count);
  if (Start_process == false && u_turning == false)
  {
    String sendThis = ""; //Declare and initialise the string we will send
    sendThis = "n3.val="; //Build the part of the string that we know
    sendThis.concat(volt); //Add the variable we want to send
    writeString(sendThis);
    delay(1000);
    if (volt < 430)
    {
      battery_low = true;
    }
    push_button();
  }
  else if (u_turning == true && (digitalRead(Emergency) == HIGH) && (obstruction  == true))
  {    
    right_rotation();
    obstruction = digitalRead(scanner);
    sensor();
    digitalWrite(red, LOW);
    digitalWrite(buzzer, LOW);
    count = 0;

    if ((signature == 1 && rotate == true && (147 <= x && x >= 167) && rotate_count >= 1000)   || (signature == 2 && rotate == true && (147 <= x && x >= 167)   && rotate_count >= 1000) || (signature == 3 && rotate == true && (147 <= x && x >= 167)  && rotate_count >= 1000))
    {
      rotate = false;
      move_slow = 1;
      rotate_count = 0;
      u_turning = false;
   
      digitalWrite(mot1_brake, HIGH);
      digitalWrite(mot2_brake, HIGH);
    digitalWrite(mot1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(mot1_reverse,HIGH);                                    
    digitalWrite(mot2_forward,HIGH);
    digitalWrite(mot2_reverse,LOW);
    }
    else if ((signature != 1) && (signature != 2) && (signature != 3) && (signature != 4) && (signature != 5) && (signature != 6) && rotate == false)
    {

      digitalWrite(mot1_brake, HIGH);
      digitalWrite(mot2_brake, HIGH);
    }
  }

  push_button();

  if ((signature != 1) && (signature != 2) && (signature != 3) && (signature != 4) && (signature != 5) && (signature != 6) && (rotate == false))
  {

    digitalWrite(mot1_brake, HIGH);
    digitalWrite(mot2_brake, HIGH);
  }

  if (Start_process == true && (digitalRead(Emergency) == HIGH) && cameraerror == 0)
  {
    obstruction = digitalRead(scanner);
    sensor();
    motion();
    
    if (obstruction == true && pause == false)
    {
      Serial.println("entering LOOPPPPPPPPPPP");
      digitalWrite(red, LOW);
      digitalWrite(buzzer, LOW);
      digitalWrite(green, HIGH);
    }
  }
  else if (Start_process == true && (digitalRead(Emergency) == HIGH) && cameraerror == 1 && rotate == true)
  {
    sensor();
    motion();
    obstruction = digitalRead(scanner);
  }
  else if (Start_process == true && (digitalRead(Emergency) == HIGH) && cameraerror == 1 && rotate == false)
  {
    Serial.println("camera_errorrrrrrrrrrrrrrrrrrrrrrrrrrr");
    digitalWrite(red, HIGH);
    digitalWrite(buzzer, HIGH);
    digitalWrite(green, LOW);
  }
}
