// Header file for motor control, distance sensor and MPU6050
#include <Servo.h>                                
#include <Adafruit_MotorShield.h>
#include "Adafruit_VL53L0X.h"                   
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "I2Cdev.h"
#include "MPU6050.h"

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
#define LED_PIN               13
#define OUTPUT_READABLE_ACCELGYRO                     // see a tab-separated list of acc X/Y/Z and gyo X/Y/Z in dec
//#define OUTPUT_BINARY_ACCELGYRO                     // much faster than UART, hard to read

// Define bluetooth (SPI) using SCK/MOSI/MISO hardware SPI pins
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// Create the VL53L0 distance sensor with the default I2C address 0x29
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Create the MPU6050 with the defualt I2C address 0x68
MPU6050 accelgyro;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Connect the two DC motor to port M1 & M2
Adafruit_DCMotor *L_Motor = AFMS.getMotor(1);       	// robot left motor
Adafruit_DCMotor *R_Motor = AFMS.getMotor(2);       	// robot right motor
Servo grab_servo;                                   	// define servo name

String readString;                                  	// Read data from UART/Bluetooth
int16_t ax, ay, az;                                   // Coordinator used by MPU 6050
int16_t gx, gy, gz;
bool blinkState = false;

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

  // MPU6050 sensor initialization
  // We have to join I2C bus first because I2C library doesn't do this
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif  

  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // Read the distance measure from VL53L0x sensor
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

    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
        delay(1000);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    
    // Read running measurement from serial port(BLE)
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
      delay(2000);
      grab_servo.write(SERVO_RELE_POS);
    }

    if (readString == "DROP"){
      grab_servo.write(SERVO_RELE_POS);
    }

    readString="";
}
