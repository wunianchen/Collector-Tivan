// Header file for motor control
#include <Servo.h>                                
#include <Adafruit_MotorShield.h>                   
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
#define SERVO_ATTACH_PIN		10						// digital#10 pin controls servo#1 input
#define	INIT_SPEED				200                     // set the initial motor speed
#define	F_SPEED					100                     // set the forward speed
#define	B_SPEED					200				       	// set the backward speed
#define	F_STOP					0					    // set the stop speed
#define	SERVO_RELE_POS			175						// set the servo release position
#define	SERVO_GRAB_POS			90						// set the servo grab position
// Constant for LIDAR distance sensor
#define VL6180X_I2C_ADDR		0x29					// the I2C address of distance sensor

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Connect the two DC motor to port M1 & M2
Adafruit_DCMotor *L_Motor = AFMS.getMotor(1);       	// robot left motor
Adafruit_DCMotor *R_Motor = AFMS.getMotor(2);       	// robot right motor
Servo grab_servo;                                   	// define servo name

String readString;                                  	// Read data from UART/Bluetooth

// Define bluetooth (SPI) using SCK/MOSI/MISO hardware SPI pins
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

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

  // Bluetooth initialization
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
}

void loop() {
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

    if (readString == "GRAB"){
      grab_servo.write(SERVO_GRAB_POS);
    }

    if (readString == "DROP"){
      grab_servo.write(SERVO_RELE_POS);
    }

    readString="";
}
