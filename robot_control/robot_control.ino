//////////////////////////////////////////////////////
//			Collector Tivan Arduino Control	V2.0		    //
//				EECS 452 Winter 2018 @ N.W				        //
//////////////////////////////////////////////////////

// TODO: Calibrate delay_per_cycle when robot is change

#include <Wire.h>
#include <Servo.h>                                
#include <Adafruit_MotorShield.h>  
#include "Adafruit_VL53L0X.h"                                  
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Constant for servo
#define SERVO_ATTACH_PIN      10                      // digital#10 pin controls servo#1 input
#define INIT_SPEED            200                     // set the initial motor speed
#define ROT_SPEED             45                      // set the rotate speed
#define F_SPEED               100                     // set the forward speed
#define F_SPEED_L             100                     // set the forward speed
#define F_SPEED_R             102                     // set the forward speed
#define B_SPEED               255                     // set the backward speed
#define F_STOP                0                       // set the stop speed
#define SERVO_RELE_POS        150                     // set the servo release position
#define SERVO_GRAB_POS        80                      // set the servo grab position
#define SERVO_GRAB_DIST_MM    68                      // define the location to grab garbage, currently set 35 mm
#define DELAY_PER_CYCLE       6450                    // measure the rotate degree, need to calibrate in the future
 
// Constant for I2C between Rpi and Arduino
#define SLAVE_ADDRESS         0x04

// Constant for accurate rotation
#define ACCURATE_ROT_DIST     250                     // determine the distance of accurate rotation
#define ACCURATE_ROT_ANGLE    -5                       // rotate degree when rotating based on sensor
#define ACCURATE_ROT_CALI     7                      // the calibration degree when rotating based on sensor    

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Connect the two DC motor to port M1 & M2
Adafruit_DCMotor *L_Motor = AFMS.getMotor(1);           // robot left motor
Adafruit_DCMotor *R_Motor = AFMS.getMotor(2);           // robot right motor
Servo grab_servo; 

// Create the VL53L0 distance sensor with the default I2C address 0x29
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Read the distance measure from VL53L0x sensor
VL53L0X_RangingMeasurementData_t VL53L0x_measure;
int last_dist_measure = 10000;
int current_dist_measure = 9999;
int measure_threshold = 450;

// I2C
char ctrl_byte;
char data_byte = 'x';
char last_data_byte = 'x';

// Accurate control variable
bool received_flag = false;                                                          // waiting the result from CV. TODO: redefine this.
bool rotate_by_sensor = false;                                                             // when the flag is true, keep doing accurate rotate by sensor
bool judge_next = false;                                                                // used in accurate rotate by sensor to decide when rotate stop

// define the robot rotate degree sequence by using tracking algorithm
double rotate_seq[] = {10, 8, 5, 3, 1.875, 1.875, 1.875, 1.875, 1.875, 1.875};
double rotate_det[] = {30,-60};
double rotate_bac[] = {10, 10, 10, 10, 8, 8, 8, 8, 8, 8};

int i;
int state = 0;

//variables of stereo
int dist_i = 0;
int dist_buffer = 0;
int distance = 0;
int back_dist = 0;
bool dist_get = false;
int forward_dist = 0;

int temp = 1;

void setup() {
  
  Serial.begin(115200);
  Serial.println("Collector Tivan Initialization Starts...");    // initialization message

  // Motor & Servo initialization
  AFMS.begin();                                                  // create with the default frequency 1.6 KHz.
  grab_servo.attach(SERVO_ATTACH_PIN);                              
  grab_servo.write(SERVO_RELE_POS);                    

  // Turn on left motor
  L_Motor -> setSpeed(INIT_SPEED);
  L_Motor -> run(RELEASE);
  
  // Turn on right motor
  R_Motor -> setSpeed(INIT_SPEED);
  R_Motor -> run(RELEASE);
  Serial.println("Motorshield Initialization Success!");
  // Motor & Servo initialization ends

  // VL53L0 sensor initialization 
  if(!lox.begin(0x30))
  {
    Serial.println("Failed to boot VL53L0X");
    while(1);
  }
  Serial.println("VL53L0X Initialization Success!"); 
  // VL53L0 sensor initialzation ends 

  // I2C Initialization
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Serial.println("I2C Initialization Success!");
}

void loop() {

  // if not detect anything
  if(ctrl_byte == 'n' && received_flag == true)
  {
    if(int(data_byte -'0') !=3)
    {
      robot_rotate(-rotate_det[int(data_byte -'0')-1]);
    }
    else
    {
      robot_rotate(-30);
      back_dist = back_dist + 15;
      robot_mov(1,15);
    }
    received_flag = false;
  }
 
  //back detection rotate
  if(ctrl_byte == 'm' && received_flag == true)
  {
    robot_rotate(-45);
    received_flag = false;
  } 
  
  // go back
  if(ctrl_byte == 'b' && received_flag == true)
  {
    robot_mov(1,back_dist-10);//robot_forward
    grab_servo.write(SERVO_RELE_POS);
    back_dist = 0;
    received_flag = false;
  }
  
  // 1. angle adjustment by tracking
  if((ctrl_byte == 'l' || ctrl_byte == 'r' || ctrl_byte == 'c' || ctrl_byte == 'e') && (last_data_byte != data_byte) && received_flag == true )
  {
    last_data_byte = data_byte;
    
    if(ctrl_byte == 'l')
    {
      Serial.println("Turn LEFT!!!");
      robot_rotate(-rotate_seq[int(data_byte -'0')-1]);
    }

    if(ctrl_byte == 'r')
    {
      Serial.println("Turn RIGHT!!!");
      robot_rotate(rotate_seq[int(data_byte -'0')-1]);
    }
    //back detection
    if(ctrl_byte == 'e')
    {
      robot_rotate(-rotate_bac[int(data_byte -'0')-1]);
    }
    if(ctrl_byte == 'c')
    {
      robot_stop();
    }
  }
  
  // 2. angle adjustment by stereo
  if(ctrl_byte == 'a' && received_flag == true)
  {
    Serial.println("ACCURATE ADJUSTEMENT!!!");
    robot_rotate(double(data_byte - '0'));//use left camera to detect & tracking
    received_flag = false;
  }
 
  // 3. receive distance
  if(ctrl_byte == 'd' && received_flag == true)
  {
    dist_i++;
    dist_buffer = dist_buffer*10 + (data_byte - '0');
    if(dist_i == 3)
    {
      dist_i = 0;
      distance = dist_buffer;
      back_dist = back_dist + distance;
      //Serial.print("final distance:");
      //Serial.println(distance);
      dist_buffer = 0;
      dist_get = true;
    } 
    received_flag = false;
  }

  // 4. robot run
  if(dist_get == true)
  {
    //Serial.print("final distance:");
    //Serial.println("step4, robot run to the target object");
    delay(1000);
    forward_dist = int(distance - 15);
    //Serial.print("forward distance:");
    //Serial.println(forward_dist);
    robot_mov(1,forward_dist);//robot_forward
    dist_get = false;
    rotate_by_sensor = true;
  }                                                            

  // 5. angle adjustment by distance sensor
  if(rotate_by_sensor == true)
  {
    delay(1000);
    robot_mov(2,0);//robot_sensor_rotate
    delay(1000);
    robot_mov(3,0);//robot_forward_and_grab
    delay(1000);
    robot_rotate(-180);
  } 
}

// Receive data from I2C
void receiveData(int byteCount) 
{
  while (Wire.available()) {
    if(i == 0)
    {
      ctrl_byte = Wire.read();
      i++;
    }
    else if(i == 1)
    {
      data_byte = Wire.read();
      i--;
      received_flag = true;  
    }
  }
}  

void robot_rotate(double angle_degree)
{
    // If angle degree is smaller than 0, turn right.
    // Left motor: FORWARD. right motor: BACKWARD.
    if(angle_degree > 0) 
    {
      L_Motor -> run(FORWARD);
      R_Motor -> run(BACKWARD);
      L_Motor -> setSpeed(ROT_SPEED);
      R_Motor -> setSpeed(ROT_SPEED);
      double delay_time = ((angle_degree/360) * DELAY_PER_CYCLE);
      delay(delay_time);
      robot_stop();
    }
    
    // If angle degree is smaller than 0, turn left.
    // Left motor: BACKWARD. right motor: FORWARD.
    if(angle_degree < 0) 
    {
      L_Motor -> run(BACKWARD);
      R_Motor -> run(FORWARD);
      L_Motor -> setSpeed(ROT_SPEED);
      R_Motor -> setSpeed(ROT_SPEED);
      double delay_time = abs((angle_degree/360) * DELAY_PER_CYCLE);
      delay(delay_time);
      robot_stop();
    }
}

void robot_stop()
{
  L_Motor -> run(RELEASE);
  R_Motor -> run(RELEASE);  
}



//******mode select
void robot_mov(int mode,int distance)
{
  if(mode==1)
  {
    robot_forward(distance);
  }
  else if(mode==2)
  {
    robot_sensor_rotate();
  }
  else if(mode==3)
  {
    robot_forward_and_grab();
  }
}

//**************mode 1
void robot_forward(int distance)
{
  
  L_Motor -> run(FORWARD);
  R_Motor -> run(FORWARD);
  L_Motor -> setSpeed(F_SPEED_L);
  R_Motor -> setSpeed(F_SPEED_R);
  double delay_t = double(distance) / 38 *2000;
  delay(delay_t);//38cm --> 2000ms
  robot_stop();
}

//**************mode 2
void robot_sensor_rotate()
{
  last_dist_measure = 10000;
  current_dist_measure = 9999;
  while(1)
  { 
    lox.rangingTest(&VL53L0x_measure, false);               // pass in "true" to get debug data printout    
    current_dist_measure = VL53L0x_measure.RangeMilliMeter;
    if(judge_next)
    {
      if((current_dist_measure > last_dist_measure))
      {
        rotate_by_sensor = false;
        judge_next = false;
        break;  
      }        
    }
    if((current_dist_measure > last_dist_measure)&&(judge_next==false)&&(last_dist_measure <= measure_threshold))//&&(judge_third==false))
    {
      judge_next = true;
    }
    
    robot_rotate(ACCURATE_ROT_ANGLE);
    last_dist_measure = current_dist_measure;  
  } 
  robot_rotate(ACCURATE_ROT_CALI);
}

//**************mode 3
void robot_forward_and_grab()
{  
  int temp = 1;
  while(1)
  {
    lox.rangingTest(&VL53L0x_measure, false);               // pass in "true" to get debug data printout

    if(VL53L0x_measure.RangeStatus !=4)
    {
      //Serial.print("Distance (mm):");
      //Serial.println(VL53L0x_measure.RangeMilliMeter);  
    }
    else
    {
      //Serial.println("Out of Range!");  
    } 
    
    L_Motor -> run(FORWARD);
    R_Motor -> run(FORWARD);
    L_Motor -> setSpeed(F_SPEED_L);
    R_Motor -> setSpeed(F_SPEED_R);
    
    //grab_servo.write(SERVO_RELE_POS); 
    if((VL53L0x_measure.RangeMilliMeter < 180)&&(temp==1))
    {
      temp = 0;
      robot_stop();
      delay(500);
      grab_servo.write(SERVO_RELE_POS);
    }
  
    if(VL53L0x_measure.RangeMilliMeter < SERVO_GRAB_DIST_MM)
    {
      robot_stop();
      delay(500);
      grab_servo.write(SERVO_GRAB_POS);
      break;
    }
  }
}


