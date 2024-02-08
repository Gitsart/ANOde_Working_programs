#include <Arduino.h>
                                                  #include <TeensyThreads.h>
#include <RPLidar.h>
#define RPLIDAR_FRONT_MOTOR 6
#define RPLIDAR_REAR_MOTOR 9
#define LED 13
#define BUMPY 18
RPLidar lidar_front, lidar_rear, lidar;

//ultrasonic
unsigned long sonar_mid_1, sonar_mid_2,sonar_left_1, sonar_left_2, sonar_right_1, sonar_right_2 = 0;
volatile bool sonar_mid_obstruct,sonar_left_obstruct, sonar_right_obstruct = false, sonar_obstruct = false;
byte data1[] = {0, 0};
byte data2[] = {0, 0};
byte data3[] = {0, 0};
int sonar_left_max = 800, sonar_right_max = 800, sonar_min = 5;
int sonar_right_count, sonar_left_count,sonar_mid_count  = 0;

// variables//
volatile bool obstruction = false;
// variables//



// custom variables //

volatile bool error = true;
int array_dist_mm[360] ;
int array_quality[360] ;
int obstacle[4] = {true, true, true, true};
int min_limit = 100;
bool stopPin = false;
bool slowPin = false;
bool errorPin = false;
int rangePin = 11;
int distRangeLow = 100, distRangeHigh = 2000;
int stopDistRange = 2000, slowDistRange = 800;
volatile float minval = 5;
int RPLIDAR, RPLIDAR_MOTOR;
volatile int min_loc = 0, bumper_val = 0, comm_error = true, min_count = 0;

// custom variables //










//function to find minimum of array elements
bool min_array(int startpt, int endpt, int min_range) {
  minval = min_range;
  int min_loc = 0;
  bool obstruction;
  for (int i = startpt; i < endpt; i++) {
    //Serial.print("value:  ");
    //Serial.print(temp[i]);
    //Serial.print("min val: ");
    //Serial.println(minval);
    if (array_dist_mm[i] < minval && array_dist_mm[i] > distRangeLow) {
      minval = array_dist_mm[i];
      min_loc = i;
      min_count++;
      minval = min(min(minval, array_dist_mm[i]), 5000);
    }
  }
    Serial.println("minimum distance:  "+(String)minval+"  location: "+(String)min_loc+"  value at minloc "+(String)array_dist_mm[min_loc]);
  if (minval > min_limit && minval < min_range && min_count > 2) {
          Serial.println("object within :"+(String)startpt+" and "+(String)endpt+"  true"+"  minval  "+minval);

    array_dist_mm[min_loc] = 0;
      Serial.println("-----------minimum distance:  "+(String)minval+"  location: "+(String)min_loc+"  value at minloc "+(String)array_dist_mm[min_loc]);
    return true;
  } else {
        Serial.println("object within :"+(String)startpt+" and "+(String)endpt+"  false"+"  minval  "+minval);
    return false;
  }

}
//function to find minimum of array elements



void lidar_thread()
{
  while (1)
  {

    lidar = lidar_front;
    RPLIDAR_MOTOR = RPLIDAR_FRONT_MOTOR;


    if (IS_OK(lidar.waitPoint()))
    {
      int distance = (int)lidar.getCurrentPoint().distance; //distance value in mm unit
      int angle    = (int)lidar.getCurrentPoint().angle; //anglue value in degree
      bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
      byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement

      //perform data processing here...
      array_dist_mm[angle] = distance;
      array_quality[angle] = quality;
      comm_error = false;
      //          Serial.println("angle:  "+(String)angle+"  distance: "+(String)distance);
      //          Serial.print(angle);
      //          Serial.print("distance: ");
      //          Serial.println(distance);

    }
    else
    {
      comm_error = true;
      analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
      digitalWrite(rangePin, LOW);
      //Serial.println("RPLidar lost");
      threads.delay(1000);
      //      SCB_AIRCR = 0x05FA0004;
      // try to detect RPLIDAR...
      rplidar_response_device_info_t info;
      if (millis() > 300000) {
        //Serial.println("resetting teensy");
        SCB_AIRCR = 0x05FA0004;
      }
      //Serial.print("laser Info:  ");
      //Serial.println(lidar.getDeviceInfo(info));
      if (IS_OK(lidar.getDeviceInfo(info, 100)))
      {
        // detected...
        lidar.startScan();
        //Serial.println("RPLidar trying to connect");
        // start motor rotating at max allowed speed
        //Serial.print("analog pin:  ");
        //Serial.println(RPLIDAR_MOTOR);
        analogWrite(RPLIDAR_MOTOR, 255);
        threads.delay(1000);
      }
    }
    threads.yield();
  }
}

bool serial_error(int strtpt, int endpt) {
  int error_count = 0;
  for (int i = 0; i < 360; i++) {
    if (array_dist_mm[i] == 0) {
      error_count++;
      //Serial.println("invaild data:  "+(String)error);
    }
  }
  if (error_count > (endpt - strtpt)) {
    return (true);
  } else {
    return (false);
  }

}

void serial_printer()
{
  for (int i = 0; i < 360; i++) {
    Serial.print("angle:  ");
    Serial.print(i);
    Serial.print("   distance: ");
    Serial.println(array_dist_mm[i]);
  }
}


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  Serial1.begin(115200);
  Serial3.begin(9600);
  Serial4.begin(9600);

  lidar_front.begin(Serial1);
  pinMode(RPLIDAR_FRONT_MOTOR, OUTPUT);
  digitalWrite(RPLIDAR_FRONT_MOTOR, HIGH);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  threads.addThread(lidar_thread);
  //  threads.delay(1000);
}

void loop() {

  bumper_val = analogRead(BUMPY);
  Serial.println(bumper_val);
  Serial3.write(0X55);
  Serial4.write(0X55);

  threads.delay(20);
  if (Serial3.available() >= 2) {
    data1[0] = Serial3.read();
    data1[1] = Serial3.read();
//    Serial3.flush();
//    Serial.print("U1 recieved data:" + String(data1[0]) + " byte2: " + String(data1[1]));
    sonar_left_1 = 256 * data1[0] + data1[1];
    sonar_left_2 = 256 * data1[1] + data1[0];
   Serial.println("  distance1: " + String(sonar_left_1) + "distance 2:" + String(sonar_left_2));
  }
  if (Serial4.available() >= 2) {
    data2[0] = Serial4.read();
    data2[1] = Serial4.read();
//    Serial4.flush();
//    Serial.print("U2 recieved data:" + String(data2[0]) + " byte2: " + String(data2[1]));
    sonar_right_1 = 256 * data2[0] + data2[1];
    sonar_right_2 = 256 * data2[1] + data2[0];
    Serial.println("  distance21: " + String(sonar_right_1) + "distance 22:" + String(sonar_right_2));
  }











  // put your main code here, to run repeatedly:
  obstacle[0] = min_array(0, 30, 700);
  obstacle[1] = min_array(330, 359, 700);
  obstacle[2] = min_array(30, 50, 500);
  obstacle[3] = min_array(310, 330, 500);
  //  obstacle[2]=serial_error(0,50);
  //  obstacle[3]=serial_error(340,359);
  //serial_printer();
//    Serial.println("obstacle 0: "+(String)obstacle[0]+"  obstacle 1:  "+(String)obstacle[1]+"  obstacle 2:  "+(String)obstacle[2]+"  obstacle 3:  "+(String)obstacle[3]+" comm error: "+String(comm_error));

  if ((20 < sonar_left_1 && sonar_left_1 < 500)|| (20 < sonar_left_2 && sonar_left_2 < 500))
  {
    Serial.println("left");
    sonar_left_count++;
    if (sonar_left_count >10)
    {
      sonar_left_obstruct = true;
      sonar_left_count = 0;
//      Serial3.write(0X55);
    }
  }
  else if ((500 < sonar_left_1 || sonar_left_1 < 20)&& (500 < sonar_left_2 || sonar_left_2 < 20))
  {
    sonar_left_obstruct = false;
    sonar_left_count = 0;
  }
    if ((20 < sonar_mid_1 && sonar_mid_1 < 500) || (20 < sonar_mid_2 && sonar_mid_2 < 500))
  {
    Serial.println("mid");
    sonar_mid_count++;
    if (sonar_mid_count > 10)
    {
      sonar_mid_obstruct = true;
      sonar_mid_count = 0;
//      Serial5.write(0X55);
    }
  }
  else if ((500 <sonar_mid_1 || sonar_mid_1 < 20) && (500 < sonar_mid_2 || sonar_mid_2 < 20))
  {
    sonar_mid_obstruct = false;
    sonar_mid_count = 0;
  }
  
   if ((20 < sonar_right_1 && sonar_right_1 < 500) || (20 < sonar_right_2 && sonar_right_2 < 500))
  {
    sonar_right_count++;
    Serial.println("right");
    if (sonar_right_count >10)
    {
      sonar_right_obstruct = true;
      sonar_right_count = 0;
//      Serial4.write(0X55);
    }
  }
  else if ((500 < sonar_right_1 || sonar_right_1 < 20) && (500 < sonar_right_2 || sonar_right_2 < 20))
  {
    sonar_right_obstruct = false;
    sonar_right_count = 0;
  }

  if (obstacle[0] == true || obstacle[1] == true || obstacle[2] == true || obstacle[3] == true || comm_error == true || sonar_left_obstruct == true || sonar_right_obstruct == true || sonar_mid_obstruct== true || bumper_val <700)
  {
    digitalWrite(LED, LOW);
    obstruction = true;
    //     Serial.println("obstruction:  ");
    threads.delay(200);
    //     SCB_AIRCR = 0x05FA0004;
  }
  else if (obstacle[0] == false && obstacle[1] == false && obstacle[2] == false && obstacle[3] == false && comm_error == false && sonar_left_obstruct == false && sonar_right_obstruct == false&&sonar_mid_obstruct== false && bumper_val>=900)
    {
      digitalWrite(LED, HIGH);
    obstruction = false;
    //     /Serial.println("no obstruction");
    min_count = 0;
  }
  threads.yield();

}
