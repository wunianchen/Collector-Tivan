// Header file for motor control and distance sensor
#include <Servo.h>                                
#include <Adafruit_MotorShield.h>
#include "Adafruit_VL53L0X.h"                   
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Header file for BLE
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLuefruitLE_UART.h"
#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

// Constant for BLE
#define FACTORYRESET_ENABLE 	1                       // enable this will put BLE in a known good state

// Constant for servo
#define SERVO_ATTACH_PIN      10                      // digital#10 pin controls servo#1 input
#define	INIT_SPEED            200                     // set the initial motor speed
#define	F_SPEED               100                     // set the forward speed
#define	B_SPEED               200                     // set the backward speed
#define	F_STOP                0					              // set the stop speed
#define	SERVO_RELE_POS        175                     // set the servo release position
#define	SERVO_GRAB_POS        90                      // set the servo grab position
#define SERVO_GRAB_DIST_MM    100                     // define the location to grab garbage, currently set 100 mm

// Constant for MPU6050
#define MPU_I2C_ADDR          0x68                    // MPU6050 I2C Address from datasheet
#define PWR_MGMT_ADDR         0x6B                    // power management register, required as start reg at MPU activation
#define PWR_MGMT_DATA         0x00                    // the data that set in MPU6050
#define ACCEL_CONFIG_ADDR     0x1C                    // acclerator configuration register address
#define ACCEL_CONFIG_DATA     0x10                    // configure the acclerator(+/-8g)
#define GYRO_CONFIG_ADDR      0x1B                    // gyro configuration register address
#define GYRO_CONFIG_DATA      0x08                    // configure the gyro(500dps as full scale)
#define ACCEL_XOUT_H_ADDR     0x3B                    // the address for ACCEL_XOUT_H reg, which is the start point of read
#define REQ_BYTES_LEN         14                      // the length of bytes once read from MPU6050 (from 3B to 48)

// Define bluetooth (SPI) using SCK/MOSI/MISO hardware SPI pins
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Create the VL53L0 distance sensor with the default I2C address 0x29
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Connect the two DC motor to port M1 & M2
Adafruit_DCMotor *L_Motor = AFMS.getMotor(1);       	  // robot left motor
Adafruit_DCMotor *R_Motor = AFMS.getMotor(2);       	  // robot right motor
Servo grab_servo;                                   	  // define servo name

// Variable used for UART/Bluetooth
String readString;                                  	  // Read data from UART/Bluetooth

// Variable for MPU6050
bool set_gyro_angles;
int gyro_X, gyro_Y, gyro_Z;                             // Gyro raw value in MPU 6050
long gyro_X_cal, gyro_Y_cal, gyro_Z_cal;                // Gyro calibration value

long acc_X, acc_Y, acc_Z, acc_total_vector;             // Acclerator raw value in MPU 6050
float angle_roll_acc, angle_pitch_acc;

float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output, angle_roll_output;

long loop_timer;
int temp;                                     

void setup() {
  Serial.begin(115200);
  Serial.println("Collector Tivan Initialization!!!");  // initialization message

  // Motor & Servo initialization
  AFMS.begin();                                         // create with the default frequency 1.6 KHz.
  grab_servo.attach(SERVO_ATTACH_PIN);                              
  grab_servo.write(SERVO_RELE_POS);                    

  // Turn on left motor
  L_Motor -> setSpeed(INIT_SPEED);
  L_Motor -> run(RELEASE);
  
  // Turn on right motor
  R_Motor -> setSpeed(INIT_SPEED);
  R_Motor -> run(RELEASE);
  // Motor & Servo initialization ends

 /* // Bluetooth initialization
  Serial.println("Adarduit Bluefruit Initializeation!!!");
  if(!ble.begin(VERBOSE_MODE))
  {
    Serial.println("Couldn't find Bluetooth");  
  }
  
  Serial.println("OK!");
  
  if(FACTORYRESET_ENABLE)
  {
    Serial.println("Performing a factory reset:");
    if(!ble.factoryReset())
    {
      Serial.println("Couldn't factory reset");  
    }  
  }

  ble.echo(false);                                          // Disable command echo from Bluefruit
  ble.info();                                               // print bluetooth information
  Serial.println("Wating for a BLE connection to continue...");
  ble.setMode(BLUEFRUIT_MODE_DATA);
  ble.verbose(false);                                       // Debug info is a little anoying after this point
  while(!ble.isConnected())
  {
    delay(5000);                                            // Wait for connection to finish  
  }
  delay(1000);                                              // Wait for connection to complete
  Serial.println("CONNECTED!");
  Serial.println("*************");
  // Bluetooth Initialization ends
*/

  // VL53L0 sensor initialization 
  if(!lox.begin())
  {
    Serial.println("Failed to boot VL53L0X");
    while(1);
  }
  Serial.println("VL53L0X Initialization Success!");
  // VL53L0 sensor initialzation ends

  // MPU6050 initialization
  Wire.begin();
  
  // Activate the MPU6050
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(PWR_MGMT_ADDR);
  Wire.write(PWR_MGMT_DATA);
  Wire.endTransmission();
  
  // Configure the acclerometer
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(ACCEL_CONFIG_ADDR);
  Wire.write(ACCEL_CONFIG_DATA);
  Wire.endTransmission(); 
  
  // Configure the gyro
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(GYRO_CONFIG_ADDR);
  Wire.write(GYRO_CONFIG_DATA);
  Wire.endTransmission(); 

  // MPU6050 Calibration
  for(int cal_int = 0; cal_int < 1000; cal_int++)
  {
    read_MPU_6050_data();                                 // read raw data from MPU6050
    gyro_X_cal += gyro_X;                                 // add the gyro x offset to the gyro_X_cal variable
    gyro_Y_cal += gyro_Y;                                 // add the gyro y offset to the gyro_Y_cal variable
    gyro_Z_cal += gyro_Z;                                 // add the gyro z offset to the gyro_Z_cal variable
    delay(3);                                             // delay 3ms to have 250Hz for-loop. TODO: might change here.
  }

  // get the average offset
  gyro_X_cal /= 1000;
  gyro_Y_cal /= 1000;
  gyro_Z_cal /= 1000;
  loop_timer = micros();
  // MPU6050 initialization ends
}

void loop() {
/*    // Read the distance measure from VL53L0x sensor
    VL53L0X_RangingMeasurementData_t VL53L0x_measure;

    Serial.print("Reading a measurement...");
    lox.rangingTest(&VL53L0x_measure, false);               // pass in "true" to get debug data printout

    if(VL53L0x_measure.RangeStatus !=4)
    {
      Serial.print("Distance (mm):"); Serial.println(VL53L0x_measure.RangeMilliMeter);  
    }
    else
    {
      Serial.println("Out of Range!");  
    }
    delay(1000); 
*/

  // Read data from MPU6050
  read_MPU_6050_data();                                    // Read the raw data from MPU6050
  
  //Subtract the offset values from the raw gyro values
  gyro_X -= gyro_X_cal;                                                
  gyro_Y -= gyro_Y_cal;                                                
  gyro_Z -= gyro_Z_cal;                                                
         
  //Gyro angle calculations . Note 0.0000611 = 1 / (250Hz x 65.5)
  angle_pitch += gyro_X * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_Y * 0.0000611;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_Z * 0.000001066);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyro_Z * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_X*acc_X)+(acc_Y*acc_Y)+(acc_Z*acc_Z));  //Calculate the total accelerometer vector
  
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  angle_pitch_acc = asin((float)acc_Y/acc_total_vector)* 57.296;       //Calculate the pitch angle
  angle_roll_acc = asin((float)acc_X/acc_total_vector)* -57.296;       //Calculate the roll angle
  
  angle_pitch_acc -= 0.0;                                              //Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;                                               //Accelerometer calibration value for roll

  if(set_gyro_angles)                                                  //If the IMU is already started
  {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;        //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  {                                                                    //At first start
    angle_pitch = angle_pitch_acc;                                     //Set the gyro pitch angle equal to the accelerometer pitch angle 
    angle_roll = angle_roll_acc;                                       //Set the gyro roll angle equal to the accelerometer roll angle 
    set_gyro_angles = true;                                            //Set the IMU started flag
  }
  
  //To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value
  Serial.print(" | Pitch_Angle  = "); Serial.println(angle_pitch_output);
  Serial.print(" | Roll_Angle  = ");  Serial.println(angle_roll_output);

 while(micros() - loop_timer < 4000);                                 //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
 loop_timer = micros();                                               // reset the loop timer
  
/*    // Read running measurement from serial port(BLE)
    // And adjust the motor speed
    while(Serial.available()){
      delay(3);
    	char c = Serial.read();
      readString += c;
    }
    
    readString.trim();
         
    if (readString == "FORWARD"){
		L_Motor -> run(FORWARD);
		R_Motor -> run(FORWARD);
		L_Motor -> setSpeed(F_SPEED);
		R_Motor -> setSpeed(F_SPEED);  
    }
    
    if (readString =="BACKWARD"){
	    L_Motor -> run(BACKWARD);
		R_Motor -> run(BACKWARD);
		L_Motor -> setSpeed(B_SPEED);
		R_Motor -> setSpeed(B_SPEED);  
    }

    if (readString =="LEFT"){
	    L_Motor -> run(FORWARD);
		R_Motor -> run(FORWARD);
		L_Motor -> setSpeed(F_STOP);
		R_Motor -> setSpeed(F_SPEED);  
    }

    if (readString =="RIGHT"){
	    L_Motor -> run(FORWARD);
		R_Motor -> run(FORWARD);
		L_Motor -> setSpeed(F_SPEED);
		R_Motor -> setSpeed(F_STOP);  
    }

    if (readString =="STOP"){
      L_Motor -> run (RELEASE);
      R_Motor -> run (RELEASE);
    }

    if (VL53L0x_measure.RangeMilliMeter < 100){                // When the distance between sensor and grab stuff
      grab_servo.write(SERVO_GRAB_POS);                       // smaller than 30, grab the stuff.
//      delay(2000);
//      grab_servo.write(SERVO_RELE_POS);
    }

    if (readString == "DROP"){
      grab_servo.write(SERVO_RELE_POS);
    }

    readString="";*/
}

void read_MPU_6050_data()
{
    // Reading raw data from MPU6050
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(ACCEL_XOUT_H_ADDR);                                       
    Wire.endTransmission();
    Wire.requestFrom(MPU_I2C_ADDR, REQ_BYTES_LEN);

    // Based on register mapping, read 14 byes data one by one
    while(Wire.available() < REQ_BYTES_LEN);
    acc_X = Wire.read()<<8|Wire.read();
    acc_Y = Wire.read()<<8|Wire.read();   
    acc_Z = Wire.read()<<8|Wire.read();
    temp  = Wire.read()<<8|Wire.read();
    gyro_X = Wire.read()<<8|Wire.read();
    gyro_Y = Wire.read()<<8|Wire.read();   
    gyro_Z = Wire.read()<<8|Wire.read();
}
