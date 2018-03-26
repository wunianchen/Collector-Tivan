////////////////////////////////////////////////////////////
//			Collector Tivan Arduino Control	V2.0		////
//				EECS 452 Winter 2018 @ N.W				////
////////////////////////////////////////////////////////////

// TODO: Communicate with Raspberry Pi
// TODO: Calibrate delay_per_cycle when robot is change

#include <Servo.h>                                
#include <Adafruit_MotorShield.h>  
#include "Adafruit_VL53L0X.h"                                  
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Constant for servo
#define SERVO_ATTACH_PIN      10                      // digital#10 pin controls servo#1 input
#define INIT_SPEED            200                     // set the initial motor speed
#define F_SPEED               100                     // set the forward speed
#define B_SPEED               100                     // set the backward speed
#define F_STOP                0                       // set the stop speed
#define SERVO_RELE_POS        150                     // set the servo release position
#define SERVO_GRAB_POS        100                     // set the servo grab position
#define SERVO_GRAB_DIST_MM    35                      // define the location to grab garbage, currently set 35 mm
#define DELAY_PER_CYCLE       4210                    // measure the rotate degree, need to calibrate in the future

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
  if(!lox.begin())
  {
    Serial.println("Failed to boot VL53L0X");
    while(1);
  }
  Serial.println("VL53L0X Initialization Success!");
  // VL53L0 sensor initialzation ends 
}

void loop() {
// TODO: This part of code is just used for testing simple motion, need to refine

  delay(10000);
  robot_rotate(90);
  delay(1000);
  robot_forward_and_grab();
  delay(1000);
  robot_rotate(180);
  delay(1000);
  robot_forward_and_release();
  robot_stop();
}

void robot_rotate(double angle_degree)
{
    // If angle degree is greater than 0, turn right.
    // Left motor: FORWARD. right motor: BACKWARD.
    if(angle_degree > 0)
    {
      L_Motor -> run(FORWARD);
      R_Motor -> run(BACKWARD);
      L_Motor -> setSpeed(F_SPEED);
      R_Motor -> setSpeed(B_SPEED);
      double test = angle_degree * DELAY_PER_CYCLE;
      Serial.println(test);
      double delay_time = (angle_degree/360) * DELAY_PER_CYCLE;
      Serial.print("angle degree is:");
      Serial.println(angle_degree);
      Serial.print("Delay Time is:");
      Serial.println(delay_time);
      Serial.println("****************");
      delay(delay_time);
    }
    
    // If angle degree is smaller than 0, turn left.
    // Left motor: BACKWARD. right motor: FORWARD.
    if(angle_degree < 0)
    {
      L_Motor -> run(BACKWARD);
      R_Motor -> run(FORWARD);
      L_Motor -> setSpeed(B_SPEED);
      R_Motor -> setSpeed(F_SPEED);
      double test = angle_degree * DELAY_PER_CYCLE;
      Serial.println(test);
      double delay_time = (angle_degree/360) * DELAY_PER_CYCLE;
      Serial.print("angle degree is:");
      Serial.println(angle_degree);
      Serial.print("Delay Time is:");
      Serial.println(delay_time);
      Serial.println("****************");
      delay(delay_time);
    }
}

void robot_stop()
{
  L_Motor -> run(RELEASE);
  R_Motor -> run(RELEASE);  
  delay(10000);
}

void robot_forward_and_release()
{
  L_Motor -> run(FORWARD);
  R_Motor -> run(FORWARD);
  L_Motor -> setSpeed(F_SPEED);
  R_Motor -> setSpeed(F_SPEED);  
  delay(5000);												// TODO: need to refine this part
  grab_servo.write(SERVO_RELE_POS);
}

void robot_forward_and_grab()
{  
  while(1)
  {
    lox.rangingTest(&VL53L0x_measure, false);               // pass in "true" to get debug data printout

    if(VL53L0x_measure.RangeStatus !=4)
    {
      Serial.print("Distance (mm):"); Serial.println(VL53L0x_measure.RangeMilliMeter);  
    }
    else
    {
      Serial.println("Out of Range!");  
    }
    
    L_Motor -> run(FORWARD);
    R_Motor -> run(FORWARD);
    L_Motor -> setSpeed(F_SPEED);
    R_Motor -> setSpeed(F_SPEED);     
  
    if(VL53L0x_measure.RangeMilliMeter < SERVO_GRAB_DIST_MM)
    {
      L_Motor -> run(RELEASE);
      R_Motor -> run(RELEASE);
      grab_servo.write(SERVO_GRAB_POS);
      break;
    }
  }
}
