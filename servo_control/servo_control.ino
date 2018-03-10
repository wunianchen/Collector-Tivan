#include <Wire.h>
#include <Servo.h>                                  // include the servo library
#include <Adafruit_MotorShield.h>                   // include the motor shield library
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BLuefruitLE_UART.h"
#include "BluefruitConfig.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

//Connect the two DC motor to port M1 & M2
Adafruit_DCMotor *L_Motor = AFMS.getMotor(1);       // robot left motor
Adafruit_DCMotor *R_Motor = AFMS.getMotor(2);       // robot right motor
Servo grab_servo;                                   // define servo name

int Init_Speed = 200;                               // set the initial motor speed
int F_Speed = 255;                                  // set the forward speed
int B_Speed = 200;				                          // set the backward speed
int F_STOP = 0;					                            // set the stop speed
String readString;                                  // Read data from UART/Bluetooth

// Define bluetooth (SPI) using SCK/MOSI/MISO hardware SPI pins
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

void setup() {
  Serial.begin(9600);
  Serial.println("Collector Tivan Initialization!!!");    // initialization message

  // Motor & Servo initialization
  AFMS.begin();                                           // create with the default frequency 1.6 KHz.
  grab_servo.attach(10);                                  // Digital 10 pin controls the Servo#1 input
  grab_servo.write(175);                                  // Servo motor starting position

  // Turn on left motor
  L_Motor -> setSpeed(Init_Speed);
  L_Motor -> run(RELEASE);
  
  // Turn on right motor
  R_Motor -> setSpeed(Init_Speed);
  R_Motor -> run(RELEASE);
  // Motor & Servo initialization ends
}


int i;

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
		L_Motor -> setSpeed(F_Speed);
		R_Motor -> setSpeed(F_Speed);  
    }
    
    if (readString =="BACKWARD"){
	    L_Motor -> run(BACKWARD);
		R_Motor -> run(BACKWARD);
		L_Motor -> setSpeed(B_Speed);
		R_Motor -> setSpeed(B_Speed);  
    }

    if (readString =="LEFT"){
	    L_Motor -> run(FORWARD);
		R_Motor -> run(FORWARD);
		L_Motor -> setSpeed(F_STOP);
		R_Motor -> setSpeed(F_Speed);  
    }

    if (readString =="RIGHT"){
	    L_Motor -> run(FORWARD);
		R_Motor -> run(FORWARD);
		L_Motor -> setSpeed(F_Speed);
		R_Motor -> setSpeed(F_STOP);  
    }

    if (readString =="STOP"){
      L_Motor -> run (RELEASE);
      R_Motor -> run (RELEASE);
    }

    if (readString == "GRAB"){
      grab_servo.write(90);
    }

    if (readString == "DROP"){
      grab_servo.write(175);
    }

    readString="";
}
