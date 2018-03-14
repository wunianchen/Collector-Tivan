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
#define REQ_BYTES_LEN         14                      // the length of bytes once read from MPU6050
#define MPU_MIN_VAL           265                     // MPU6050 minimum value
#define MPU_MAX_VAL           402                     // MPU6050 maximum value

// Define bluetooth (SPI) using SCK/MOSI/MISO hardware SPI pins
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Create the VL53L0 distance sensor with the default I2C address 0x29
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Connect the two DC motor to port M1 & M2
Adafruit_DCMotor *L_Motor = AFMS.getMotor(1);       	  // robot left motor
Adafruit_DCMotor *R_Motor = AFMS.getMotor(2);       	  // robot right motor
Servo grab_servo;                                   	  // define servo name

String readString;                                  	  // Read data from UART/Bluetooth
int16_t AcX, AcY, AcZ;                                  // Acclerator value in MPU 6050
int16_t GyX, GyY, GyZ;                                  // Gyro value in MPU 6050
double Angle_X_Deg, Angle_Y_Deg, Angle_Z_Deg;           // the final angle value on each axis                                      

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
  Wire.beginTransmission(MPU_I2C_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
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

    // Reading data (in degree) from MPU6050
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x3B);                                       // TODO: parameterize these instruction
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADDR, REQ_BYTES_LEN, true);   
    AcX = Wire.read()<<8|Wire.read();
    AcY = Wire.read()<<8|Wire.read();   
    AcZ = Wire.read()<<8|Wire.read();

    int xAng = map(AcX, MPU_MIN_VAL, MPU_MAX_VAL, -90, 90);
    int yAng = map(AcY, MPU_MIN_VAL, MPU_MAX_VAL, -90, 90);
    int zAng = map(AcZ, MPU_MIN_VAL, MPU_MAX_VAL, -90, 90);

    Angle_X_Deg = RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
    Angle_Y_Deg = RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
    Angle_Z_Deg = RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

    Serial.print("Angle_X = ");
    Serial.println(Angle_X_Deg);

    Serial.print("Angle_Y = ");
    Serial.println(Angle_Y_Deg);

    Serial.print("Angle_Z = ");
    Serial.println(Angle_Z_Deg);
    Serial.println("------------------------------------------------------");
    delay(500);

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
