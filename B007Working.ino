///signature 1 and 2 for speed              ---- green
// signature 3 is slow                      ---- not used
// signature 4 is for pause ,operator acknowledge, continue straight --- not used
// signature 5 is for  pause operator acknowlege,rotate              --- red
// signature 6 is to rotate                                          --- yellow

//com port 18




#include <Pixy2.h>
Pixy2 pixy;
#include "Pixy2I2C.h"
  Pixy2I2C pixyserial;

//input outputs//


int red = A3;
int green = A4;
int front_pb = A1;
int Emergency = 3;
int motor1_forward = 5;
int motor1_reverse = 4;
int motor1_brake = A13;
int motor1_speed = 7;
int motor2_reverse = 9;
int motor2_forward = 11;
int motor2_brake = A15;
int motor2_speed = 8;
int voltage_input = A0;
int front_scanner = 10;
int back_pb = 2;
int back_scanner = 13;
//input outputs//



int period = 30;
int previousMillis = 0;


/// signature 1 will be affected if high speed , medium speed and low speed setting are changed//
// signature 3 will only depend on slow speed settings//
// speed variables 
int count_lowspeed=200;
int count_medspeed=400;
int count_highspeed=600;

int lidarcentre_front = 155;
int lidarcentre_back = 151;

//high speed setting//
int high_spd =40;//40     
int high_spd_high =45;// 
int high_spd_low=35;//30

//medium speed setting//
int med_spd =25;//
int med_spd_high = 30;//50
int med_spd_low = 20;//

//slow speed setting//
int slow_spd =  12;    //was20
int slow_spd_high = 14;  //was30
int slow_spd_low =10; //was10

int count_slow = 100;
int count_high =200;


//Variables//
bool lidar_out = false;
bool off_path = false;
bool battery_low = false;
bool pause = false;
bool op_ack = false;
bool emg = false;
bool front_obstruction   = LOW;
bool back_obstruction   = LOW;
bool Start_process = false;
bool Back_Start_process = false;
bool front_cam = false;
bool back_cam = false;

int i, j, l, k = 0;
int distance, distance_1, distance_2 = 0;
int signature, signature_b = 0;
int x, x_b = 0;                     //positon x axis
int y, y_b = 0;                     //position y axis
unsigned int width_1 = 0;         //object's width_1
unsigned int height_1 = 0;        //object's height_1
int map_1, map_2 = 0;
int right_turn = 0;
int move_slow = 0;
int count = 0;
int cameraerror = 0;
int rotate_count = 0;
int val = 0;
int volt = 0;
char buf[32];                 //positon x axis


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

void front_cam_loop()
{
  uint16_t blocks;
  blocks = pixy.ccc.getBlocks();//receive data from pixy
  signature = pixy.ccc.blocks[i].m_signature;    //get object's signature
  x = pixy.ccc.blocks[i].m_x;                    //get x position
  y = pixy.ccc.blocks[i].m_y;                    //get y position
  width_1 = pixy.ccc.blocks[i].m_width;            //get width_1
  height_1 = pixy.ccc.blocks[i].m_height;          //get height_1
  int i;

  if (pixy.ccc.numBlocks)
  {
    cameraerror = 0;
  }
  else
  {
    cameraerror = 1;
  }
  if ((signature != 1) && (signature != 2) && (signature != 3) && (signature != 4) && (signature != 5) && (signature != 6))
  {
   
    digitalWrite(motor1_brake, HIGH);
    digitalWrite(motor2_brake, HIGH);
    count = 0;
   move_slow=1;
  }
  else if (((signature == 1) || (signature == 2) || (signature == 3)) || (signature == 4) || (signature == 5) || (signature == 6) && cameraerror == 1)
  {
    cameraerror = 0;
  }
  if (lidar_out == true)
  {
    move_slow = 1;
    count = 0;
  }
}


void back_cam_loop()
{
  uint16_t blocks;
  blocks = pixyserial.ccc.getBlocks();
  signature_b = pixyserial.ccc.blocks[i].m_signature;    //get object's signature
  x_b = pixyserial.ccc.blocks[i].m_x;                    //get x position
  y_b = pixyserial.ccc.blocks[i].m_y;


  if (pixyserial.ccc.numBlocks)
  {
    cameraerror = 0;
   }

  else
  {
    cameraerror = 1;
  }

  if ((signature_b != 1) && (signature_b != 2) && (signature_b != 3) && (signature_b != 4) && (signature_b != 5) && (signature_b != 6))
  {
  
      digitalWrite(motor1_brake, HIGH);
    digitalWrite(motor2_brake, HIGH);
    count = 0;
    move_slow=1;
   
  }
  else if (((signature_b == 1) || (signature_b == 2) || (signature_b == 3)) || (signature_b == 4) || (signature_b == 5) || (signature_b == 6) )
  {
    cameraerror = 0;
  }
  if (lidar_out == true)
  {
    move_slow = 1;
    count = 0;
  }
}
void front_sensor()
{
  if (front_obstruction   == false)
  {
    lidar_out = true;

    digitalWrite(motor1_brake, HIGH);
    digitalWrite(motor2_brake, HIGH);

    move_slow = 1;
    front_obstruction   = digitalRead(front_scanner);
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
    move_slow = 1;
    count = 0;
  }
  else
  {
    front_obstruction   = digitalRead(front_scanner);
    if ((front_obstruction   == true ) && (lidar_out == true)&& pause == false)
    {
      move_slow = 1;
      count = 0;
      lidar_out = false;
      delay(100);
 
        digitalWrite(motor1_brake, LOW);
      digitalWrite(motor2_brake, LOW);
    
    }
  }
}

void back_sensor()
{
  if (back_obstruction   == false)
  {
    lidar_out = true;

    digitalWrite(motor1_brake, HIGH);
    digitalWrite(motor2_brake, HIGH);
  
    move_slow = 1;
    back_obstruction   = digitalRead(back_scanner);
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
    move_slow = 1;
    count = 0;
  }
  else
  {
    back_obstruction   = digitalRead(back_scanner);
    if ((back_obstruction   == true ) && (lidar_out == true) && pause == false)
    {
      move_slow = 1;
      count = 0;
      lidar_out = false;
      delay(100);
 
      digitalWrite(motor1_brake, LOW);
      digitalWrite(motor2_brake, LOW);
    
    }
  }
}
void push_button()
{
  if  ((digitalRead(Emergency) == HIGH) && (digitalRead(front_pb) == LOW) && (digitalRead(back_pb) == HIGH) && battery_low == false && Start_process == false && pause == false && Back_Start_process == false )
  {
    front_cam = true;
    back_cam = false;
    count = 0;
    op_ack = false;
    Start_process = true;
    move_slow = 1;
  }
  else if ((digitalRead(Emergency) == HIGH) && (digitalRead(front_pb) == HIGH) && (digitalRead(back_pb) == LOW)  && battery_low == false && Back_Start_process == false && pause == false && Start_process == false)
  {
    front_cam = false;
    back_cam = true;
    count = 0;
    op_ack = false;
    Back_Start_process = true;
    move_slow = 1;
  }

  else if  ((digitalRead(Emergency) == HIGH) && (digitalRead(front_pb) == LOW) && (emg == true) && pause == false && Start_process == true)
  {
    Serial.println("emg release front pbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
    emg = false;
    back_cam = true;  
    front_cam = false; 
  }
  else if  ((digitalRead(Emergency) == HIGH) && (digitalRead(back_pb) == LOW) && (emg == true)&& pause == false && Back_Start_process == true)
  {
     Serial.println("emg release back pbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb");
    emg = false;
    front_cam = true;
    back_cam = false; 
  }
  else if(emg == false && Back_Start_process == true && pause == false && back_cam == false)
  {
   back_cam = true;
   front_cam = false;
  }
  else if(emg == false && Start_process == true && pause == false && front_cam == false)
  {
  front_cam = true;
  back_cam = false;
  }
  else if(emg == false && Back_Start_process == true && pause == false && back_cam == true && front_cam == true)
  {
   back_cam = true;
   front_cam = false;
  }
  else if(emg == false && Start_process == true && pause == false && back_cam == true && front_cam == true)
  {
   front_cam = true;
   back_cam = false;
  }
  else if  ((digitalRead(Emergency) == HIGH) && (digitalRead(front_pb) == LOW) && (pause == true) && Back_Start_process == true)
  {
    emg = false;
    op_ack = true;
     pause = false;
    back_cam = true;
    Serial.println("front_op_ack");
  }
  else if  ((digitalRead(Emergency) == HIGH) && (digitalRead(back_pb) == LOW) && (pause == true) && Start_process == true )
  {
    emg = false;
     pause = false;
    op_ack = true;
    front_cam = true;
     Serial.println("back_op_ack");
  }
  else if (digitalRead(Emergency) == LOW)
  {
    Serial.println("emg pressssssssssssssssssssssssssssed");
    count = 0;
    move_slow = 1;
    op_ack = false;
    emg = true;
    back_cam = false;  //previosly false 
    front_cam = false; // previously false
    //digitalWrite(motor1_on, LOW);
    //digitalWrite(motor2_on, LOW);
    digitalWrite(motor1_brake, HIGH);
    digitalWrite(motor2_brake, HIGH);
      digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
      digitalWrite(motor1_reverse,LOW);//wasHIGH
        digitalWrite(motor2_forward,LOW);//wasHIGH
        digitalWrite(motor2_reverse,LOW);
          digitalWrite(red, HIGH);
          digitalWrite(green, LOW);
  }
}

void forward_motion()
{
  if (signature == 1)
  {
    pause = false;
    }
  Serial.println("entering forward motion");
  //  Serial.println(pause);
  //   Serial.println(move_slow);
  //  Serial.println(signature);
  //   Serial.println(front_obstruction);
  //    Serial.println(lidar_out);
  if (signature == 1 && (front_obstruction    == true)  && lidar_out == false && move_slow == 1 && pause == false)
  {
     Serial.println("yellow forward loop");
    if (0 <= count && count <= count_slow)
    {
      if (0 <= x && x <= lidarcentre_front)
      {
        cameraerror = 0;
        rotate_count = 0;
        op_ack = false;
        pause = false;
        map_2 = map(x, lidarcentre_front, 0, slow_spd, slow_spd_high);
        map_1 = map(x, lidarcentre_front, 0, slow_spd, slow_spd_low);
        analogWrite(motor1_speed, map_1);
        analogWrite(motor2_speed, map_2);
        digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
        digitalWrite(motor1_reverse,HIGH);
        digitalWrite(motor2_forward,HIGH);
        digitalWrite(motor2_reverse,LOW);
        digitalWrite(motor1_brake, LOW);
        digitalWrite(motor2_brake, LOW);
                move_slow = 1;
        count++;
      }
      else if (( lidarcentre_front < x && x <= 314))
      {
        cameraerror = 0;
        rotate_count = 0;
        op_ack = false;
        pause = false;
        map_2 = map(x, lidarcentre_front, 314, slow_spd, slow_spd_low);
        map_1 = map(x, lidarcentre_front, 314, slow_spd, slow_spd_high);
       
        analogWrite(motor1_speed, map_1);
        analogWrite(motor2_speed, map_2);
        digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
        digitalWrite(motor1_reverse,HIGH);
        digitalWrite(motor2_forward,HIGH);
        digitalWrite(motor2_reverse,LOW);
        digitalWrite(motor1_brake, LOW);
        digitalWrite(motor2_brake, LOW);
             move_slow = 1;
        count++;
      }
    }

    else if (count_slow<count && count<=count_high)
    {
      if (0 <= x && x <= lidarcentre_front)
      {
        cameraerror = 0;
        rotate_count = 0;
        op_ack = false;
        pause = false;
        map_2 = map(x, lidarcentre_front, 0, med_spd, med_spd_high);
        map_1 = map(x, lidarcentre_front, 0, med_spd, med_spd_low);
    
        analogWrite(motor1_speed, map_1);
        analogWrite(motor2_speed, map_2);
        digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
        digitalWrite(motor1_reverse,HIGH);
        digitalWrite(motor2_forward,HIGH);
        digitalWrite(motor2_reverse,LOW);
        digitalWrite(motor1_brake, LOW);
        digitalWrite(motor2_brake, LOW);
               move_slow = 1;
        count++;
      }
      else if (( lidarcentre_front < x && x <= 314))
      {
        cameraerror = 0;
        rotate_count = 0;
        op_ack = false;
        pause = false;
        map_2 = map(x, lidarcentre_front, 314, med_spd, med_spd_low);
        map_1 = map(x, lidarcentre_front, 314, med_spd, med_spd_high);
       
        analogWrite(motor1_speed, map_1);
        analogWrite(motor2_speed, map_2);
        digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
        digitalWrite(motor1_reverse,HIGH);
        digitalWrite(motor2_forward,HIGH);
        digitalWrite(motor2_reverse,LOW);
        digitalWrite(motor1_brake, LOW);
        digitalWrite(motor2_brake, LOW);
               move_slow = 1;
        count++;
      }
    } 
    else if (count > count_high)
    {
      move_slow = 0;
      count = 0;
    }
  }

  else if (signature == 1 && (front_obstruction == true)  && lidar_out == false && move_slow == 0 && pause == false )
  {
     Serial.println("yellow forward fast");
    if (0 <= x && x <= lidarcentre_front)
    {
      cameraerror = 0;
      rotate_count = 0;
      op_ack = false;
      pause = false;
      map_2 = map(x, lidarcentre_front, 0, high_spd, high_spd_high);
      map_1 = map(x, lidarcentre_front, 0, high_spd, high_spd_low);

      analogWrite(motor1_speed, map_1);
      analogWrite(motor2_speed, map_2);
       digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
        digitalWrite(motor1_reverse,HIGH);
        digitalWrite(motor2_forward,HIGH);
        digitalWrite(motor2_reverse,LOW);
       digitalWrite(motor1_brake, LOW);
      digitalWrite(motor2_brake, LOW);
         }
    else if (( lidarcentre_front < x && x <= 314))
    {
      cameraerror = 0;
      rotate_count = 0;
      op_ack = false;
      pause = false;
      map_2 = map(x, lidarcentre_front, 310, high_spd, high_spd_low);
      map_1 = map(x, lidarcentre_front, 314, high_spd, high_spd_high);
      analogWrite(motor1_speed, map_1);
      analogWrite(motor2_speed, map_2);
         digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
        digitalWrite(motor1_reverse,HIGH);
        digitalWrite(motor2_forward,HIGH);
        digitalWrite(motor2_reverse,LOW); 
      digitalWrite(motor1_brake, LOW);
      digitalWrite(motor2_brake, LOW);
      
    }
  }

  else if ((signature == 3)&& (front_obstruction == true)  && lidar_out == false  && (lidarcentre_front < x && x < 314)&& pause == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    op_ack = false;
    pause = false;
    move_slow = 1;
    map_2 = map(x, lidarcentre_front, 310, (slow_spd), slow_spd_low); //lidarcentre_front&314
    map_1 = map(x, lidarcentre_front, 314, (slow_spd), slow_spd_high);
    analogWrite(motor1_speed, map_1);
    analogWrite(motor2_speed, map_2);
      digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
      digitalWrite(motor1_reverse,HIGH);
      digitalWrite(motor2_forward,HIGH);
      digitalWrite(motor2_reverse,LOW);
      digitalWrite(motor1_brake, LOW);
      digitalWrite(motor2_brake, LOW);
    
  }

  else if ((signature == 3) && (front_obstruction == true)  && lidar_out == false && ( 0 <= x && x <= lidarcentre_front)&& pause == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    op_ack = false;
    pause = false;
    move_slow = 1;
    map_2 = map(x, lidarcentre_front, 0, (slow_spd), slow_spd_high);
    map_1 = map(x, lidarcentre_front, 0, (slow_spd), slow_spd_low);
    analogWrite(motor1_speed, map_1);
    analogWrite(motor2_speed, map_2);
    digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
    digitalWrite(motor1_reverse,HIGH);
    digitalWrite(motor2_forward,HIGH);
    digitalWrite(motor2_reverse,LOW);
    digitalWrite(motor1_brake, LOW);
    digitalWrite(motor2_brake, LOW);
  
  }


  else if ((signature == 4) && (front_obstruction  == true) && (pause == false) && lidar_out == false && Start_process == true)
  {
    cameraerror = 0;
    rotate_count = 0;
    count = 0;
    pause = true;
    delay(100);
    analogWrite(motor1_speed, map_1);
    analogWrite(motor2_speed, map_2);

    digitalWrite(motor1_brake, HIGH);
    digitalWrite(motor2_brake, HIGH);
      digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
      digitalWrite(motor1_reverse,HIGH);
        digitalWrite(motor2_forward,HIGH);
        digitalWrite(motor2_reverse,LOW);
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
    move_slow = 1;
    Serial.println("entered 4");
  }
 else if ((signature == 4) && (pause == true) && Start_process == true && op_ack == false)
   {
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
    Serial.println("entered 4a pause");
    move_slow = 1;
   }

  else if ((signature == 4) && (pause == true) && Start_process == true && op_ack == true)
  {
    move_slow = 1;
    Start_process = false;
    front_cam = false;
    back_cam = true;
    Back_Start_process = true;
    Serial.println("entered 4aaaaaaaa");
     digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
       digitalWrite(motor1_reverse,LOW); 
       digitalWrite(motor2_forward,LOW);
      digitalWrite(motor2_reverse,HIGH); 
    pause = false;
    op_ack = false;
  }
    else if ((signature == 5) && ( 0 <= x && x <= lidarcentre_front) && (front_obstruction  == true)  && (pause == false) && lidar_out == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    pause = true;
  digitalWrite(motor1_brake, HIGH);
  digitalWrite(motor2_brake, HIGH);
   digitalWrite(motor1_forward,LOW); 
  digitalWrite(motor1_reverse,LOW); 
  digitalWrite(motor2_forward,LOW);
  digitalWrite(motor2_reverse,LOW);
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
        move_slow = 1;
  }
  else if ((signature == 5) && (lidarcentre_front < x && x <= 314) && (front_obstruction  == true)  && (pause == false) && lidar_out == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    pause = true;
  
    digitalWrite(motor1_brake, HIGH);
  digitalWrite(motor2_brake, HIGH);
   digitalWrite(motor1_forward,LOW); 
  digitalWrite(motor1_reverse,LOW); 
  digitalWrite(motor2_forward,LOW);
  digitalWrite(motor2_reverse,LOW);
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
   
     move_slow = 1;
  }
  else if ((signature == 5) && ( 0 <= x && x <= lidarcentre_front) && (front_obstruction  == true)  && (pause == true) && lidar_out == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    analogWrite(motor1_speed, map_1);
    analogWrite(motor2_speed, map_2);
 
      digitalWrite(motor1_brake,LOW);
    digitalWrite(motor2_brake, LOW);
    digitalWrite(motor1_forward,LOW); 
    digitalWrite(motor1_reverse,HIGH);                                    
    digitalWrite(motor2_forward,HIGH);
    digitalWrite(motor2_reverse,LOW); 
       move_slow = 1;
  }
  else if ((signature == 5) && (lidarcentre_front < x && x <= 314) && (front_obstruction  == true) && (pause == true) && lidar_out == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    analogWrite(motor1_speed, map_1);
    analogWrite(motor2_speed, map_2);
    digitalWrite(motor1_brake, LOW);
    digitalWrite(motor2_brake, LOW);
    digitalWrite(motor1_forward,LOW); 
    digitalWrite(motor1_reverse,HIGH);                                    
    digitalWrite(motor2_forward,HIGH);
    digitalWrite(motor2_reverse,LOW); 
    move_slow = 1;
  }
}
void reverse_motion()
{
  if (signature_b == 1)
  {
    pause = false;
    }
  Serial.println("entering reverse loop");
  // Serial.print(signature_b ); Serial.print(back_obstruction ); Serial.print(lidar_out); Serial.print(move_slow); Serial.println(pause); 
  if (signature_b == 1 && (back_obstruction    == true) && lidar_out == false && move_slow == 1 && pause == false)
  {
    if (0 <= count && count <= count_slow)
    {
      if (0 <= x_b && x_b <= lidarcentre_back)
      {
        cameraerror = 0;
        rotate_count = 0;
        op_ack = false;
        pause = false;
        map_1 = map(x_b, lidarcentre_back, 0, slow_spd, slow_spd_high);
        map_2 = map(x_b, lidarcentre_back, 0, slow_spd, slow_spd_low);
      
        analogWrite(motor1_speed, map_1);
        analogWrite(motor2_speed, map_2);
       digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
       digitalWrite(motor1_reverse,LOW); 
       digitalWrite(motor2_forward,LOW);
         digitalWrite(motor2_reverse,HIGH); 
          digitalWrite(motor1_brake, LOW);
        digitalWrite(motor2_brake, LOW);
        move_slow = 1;
        count++;
      }
      else if (( lidarcentre_back < x_b && x_b <= 314))
      {
        cameraerror = 0;
        rotate_count = 0;
        op_ack = false;
        pause = false;
        map_1 = map(x_b, lidarcentre_back, 314, slow_spd, slow_spd_low);
        map_2 = map(x_b, lidarcentre_back, 314, slow_spd, slow_spd_high);
        analogWrite(motor1_speed, map_1);
        analogWrite(motor2_speed, map_2);
         digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
        digitalWrite(motor1_reverse,LOW); 
       digitalWrite(motor2_forward,LOW);
        digitalWrite(motor2_reverse,HIGH); 
         digitalWrite(motor1_brake, LOW);
        digitalWrite(motor2_brake, LOW);
        move_slow = 1;
        count++;
      }
    }

    else if (count_slow < count && count <= count_high)
    {
      if (0 <= x_b && x_b <= lidarcentre_back)
      {
        cameraerror = 0;
        rotate_count = 0;
        op_ack = false;
        pause = false;
        map_1 = map(x_b, lidarcentre_back, 0, med_spd, med_spd_high);
        map_2 = map(x_b, lidarcentre_back, 0, med_spd, med_spd_low);
    
        analogWrite(motor1_speed, map_1);
        analogWrite(motor2_speed, map_2);
       digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
       digitalWrite(motor1_reverse,LOW); 
       digitalWrite(motor2_forward,LOW);
       digitalWrite(motor2_reverse,HIGH); 
         digitalWrite(motor1_brake, LOW);
        digitalWrite(motor2_brake, LOW);
        move_slow = 1;
        count++;
      }
      else if (( lidarcentre_back < x_b && x_b <= 314))
      {
        cameraerror = 0;
        rotate_count = 0;
        op_ack = false;
        pause = false;
        map_1 = map(x_b, lidarcentre_back, 314, med_spd, med_spd_low);
        map_2 = map(x_b, lidarcentre_back, 314, med_spd, med_spd_high);
        
        analogWrite(motor1_speed, map_1);
        analogWrite(motor2_speed, map_2);
          digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
          digitalWrite(motor1_reverse,LOW); 
          digitalWrite(motor2_forward,LOW);
          digitalWrite(motor2_reverse,HIGH); 
         digitalWrite(motor1_brake, LOW);
        digitalWrite(motor2_brake, LOW);
        move_slow = 1;
        count++;
      }
    }
    else if (count > count_high)
    {
      move_slow = 0;
      count = 0;
    }

  }
  else if (signature_b == 1 && (back_obstruction    == true) && lidar_out == false && move_slow == 0 && pause == false)
  {
    if (0 <= x_b && x_b <= lidarcentre_back)
    {
      cameraerror = 0;
      rotate_count = 0;
      op_ack = false;
      pause = false;
      map_1 = map(x_b, lidarcentre_back, 0, high_spd, high_spd_high);
      map_2 = map(x_b, lidarcentre_back, 0, high_spd, high_spd_low);
      
      analogWrite(motor1_speed, map_1);
      analogWrite(motor2_speed, map_2);
      digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
      digitalWrite(motor1_reverse,LOW); 
      digitalWrite(motor2_forward,LOW);
      digitalWrite(motor2_reverse,HIGH); 
       digitalWrite(motor1_brake, LOW);
      digitalWrite(motor2_brake, LOW);
  
    }
    else if (( lidarcentre_back < x_b && x_b <= 314))
    {
      cameraerror = 0;
      rotate_count = 0;
      op_ack = false;
      pause = false;
      map_1 = map(x_b, lidarcentre_back,314, high_spd, high_spd_low);
      map_2 = map(x_b, lidarcentre_back,314, high_spd, high_spd_high);
      
      analogWrite(motor1_speed, map_1);
      analogWrite(motor2_speed, map_2);
       digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
       digitalWrite(motor1_reverse,LOW); 
      digitalWrite(motor2_forward,LOW);
      digitalWrite(motor2_reverse,HIGH); 
        digitalWrite(motor1_brake, LOW);
      digitalWrite(motor2_brake, LOW);
      
    }
  }

  else if ((signature_b == 3)&& (back_obstruction    == true) && lidar_out == false && (0 < x_b && x_b < lidarcentre_back)&& pause == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    op_ack = false;
    pause = false;
    move_slow = 1;
    map_1 = map(x_b, lidarcentre_back, 0, slow_spd, slow_spd_high);
    map_2 = map(x_b, lidarcentre_back, 0, slow_spd, slow_spd_low);
  
    analogWrite(motor1_speed, map_1);
    analogWrite(motor2_speed, map_2);
  digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
  digitalWrite(motor1_reverse,LOW); 
  digitalWrite(motor2_forward,LOW);
  digitalWrite(motor2_reverse,HIGH); 
      digitalWrite(motor1_brake, LOW);
    digitalWrite(motor2_brake, LOW);
  }
  else if ((signature_b == 3) && (back_obstruction    == true) && lidar_out == false&& ( lidarcentre_back < x_b && x_b <= 314)&& pause == false)
  {
    cameraerror = 0;
    rotate_count = 0;
    op_ack = false;
    pause = false;
    move_slow = 1;
    map_1 = map(x_b, lidarcentre_back, 314, slow_spd, slow_spd_low);
    map_2 = map(x_b, lidarcentre_back, 314, slow_spd, slow_spd_high);
   
    analogWrite(motor1_speed, map_1);
    analogWrite(motor2_speed, map_2);
    digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
  digitalWrite(motor1_reverse,LOW); 
  digitalWrite(motor2_forward,LOW);
  digitalWrite(motor2_reverse,HIGH); 
     digitalWrite(motor1_brake, LOW);
    digitalWrite(motor2_brake, LOW);
  
  }


  else if ((signature_b == 4) && (back_obstruction  == true) && (pause == false) && lidar_out == false && Back_Start_process == true)
  {
    cameraerror = 0;
    rotate_count = 0;
    pause = true;
    count = 0;
    analogWrite(motor1_speed, map_1);
    analogWrite(motor2_speed, map_2);
    delay(100);
     digitalWrite(motor1_forward,HIGH); //(LOW = 5V , HIGH = 0V )
     digitalWrite(motor1_reverse,LOW); 
     digitalWrite(motor2_forward,LOW);
     digitalWrite(motor2_reverse,HIGH); 
      digitalWrite(motor1_brake, HIGH);
    digitalWrite(motor2_brake, HIGH);
  
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
    Serial.println("entered 4b");
    move_slow = 1;
  }

   else if ((signature_b == 4) && (pause == true) && Back_Start_process == true && op_ack == false)
   {
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
    Serial.println("entered 4b pause");
    move_slow = 1;
   }
  

  else if ((signature_b == 4) && (pause == true) && Back_Start_process == true && op_ack == true)
  {
    move_slow = 1;
    Back_Start_process = false;
    Start_process = true;
    back_cam = false;
    front_cam = true;
    Serial.println("entered 4bbbbbbbbbbbbbbbbbbbbb");
       digitalWrite(motor1_forward,LOW); //(LOW = 5V , HIGH = 0V )
       digitalWrite(motor1_reverse,HIGH);
       digitalWrite(motor2_forward,HIGH);
       digitalWrite(motor2_reverse,LOW);
    pause = false;
    op_ack = false;
  }
}

void setup()
{
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  Serial1.begin(9600);// DISPLAY
  pixy.init();
  pixyserial.init();
  pinMode(motor1_reverse, OUTPUT);
  pinMode(motor2_reverse, OUTPUT);
    pinMode(motor1_brake, OUTPUT);
  pinMode(motor2_brake, OUTPUT);
  pinMode(motor1_forward, OUTPUT);
   pinMode(motor2_forward, OUTPUT);
  pinMode(motor1_speed, OUTPUT);
  pinMode(motor2_speed, OUTPUT);
  pinMode(red, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(front_pb, INPUT_PULLUP);
  pinMode(Emergency, INPUT_PULLUP);
  pinMode(back_pb, INPUT_PULLUP);
  pinMode(voltage_input, INPUT);
  pinMode(front_scanner, INPUT);
  pinMode(back_scanner, INPUT);
  digitalWrite(red, LOW);
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
  
  Serial.print("emg"); Serial.println(emg);
  Serial.print(Start_process); 
  Serial.println(Back_Start_process);
  Serial.println(cameraerror);                                              
  Serial.print(front_cam);
  Serial.println(back_cam);
  Serial.print("pause");Serial.println(pause);
  Serial.print("op_ack");Serial.println(op_ack);
  Serial.print("signature"); Serial.println(signature);
  Serial.print("signature_b");Serial.println(signature_b);
//  Serial.print(digitalRead(front_pb));Serial.print(digitalRead(Emergency));
//    Serial.println(digitalRead(back_pb));
    
 Serial.print("speed_1 : ");Serial.println(map_1);
 Serial.print("speed_2 : ");Serial.println(map_2);
 Serial.print("move_slow : ");Serial.println(move_slow);
  Serial.print("count : ");Serial.println(count);

//  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= period)
  {
    previousMillis = currentMillis;
    if (front_cam == true)
    {
      front_cam_loop();
    }
    else if (back_cam == true)
    {
      back_cam_loop();
    }
  }
  front_obstruction   = digitalRead(front_scanner);
  back_obstruction  = digitalRead(back_scanner);
  Serial.print("front_obstruction");Serial.println(front_obstruction);
  Serial.print("back_obstruction");Serial.println(back_obstruction);
  volt = analogRead(voltage_input);

  if (Start_process == false && Back_Start_process == false)
  {
    if (volt < 400)
    {
      battery_low = true;
    }
    push_button();
  }

  push_button();

  if (Start_process == true && (emg == false) && cameraerror == 0 && front_cam == true)
  {
    Serial.println("enterring main front loop");
    front_sensor();
    forward_motion();
    if (front_obstruction   == true && pause == false)
    {
      digitalWrite(red, LOW);
      digitalWrite(green, HIGH);
    }
  }
  else if (Back_Start_process == true && (emg == false) && cameraerror == 0 && back_cam == true)
  {
    Serial.println("enterring main back loop");
    back_sensor();
    reverse_motion();
    if (back_obstruction   == true && pause == false)
    {
      digitalWrite(red, LOW);
      digitalWrite(green, HIGH);
    }
  }

  else if (Start_process == true && (emg == false) && cameraerror == 1)
  {
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
  }
  else if (Back_Start_process == true && (emg == false) && cameraerror == 1)
  {
    digitalWrite(red, HIGH);
    digitalWrite(green, LOW);
  }
}
