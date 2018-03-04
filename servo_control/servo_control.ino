#include <AFMotor.h>                            // include the motor shield library
#include <Servo.h>                              // include the servo library

#define rightSpeed 160                          // The speed ranges from 0 (stopped) to 255 (full speed)
#define leftSpeed 110

AF_DCMotor leftMotor(1, MOTOR12_8KHZ);          // Create the AF_DCMotor Object. The first parameter: port number
AF_DCMotor rightMotor(2, MOTOR12_8KHZ);         // Frequency: how fast the spead controlling signal is.

Servo servo1;                                   // Define servo name

String readString;

void setup() {
  Serial.begin(9600);
  servo1.attach(10);                            // Digital 10 pin controls the Servo#1 input
  servo1.write(175);                            // Servo motor starting position
  
  rightMotor.setSpeed(rightSpeed);
  leftMotor.setSpeed(leftSpeed);
}

void loop() {
  // TODO: Get character from bluetooth module
  while(Serial.available()){
    delay(50);
    char c=Serial.read();
    readString+=c;
  }
  if(readString.length()>0){
    Serial.println(readString);

    if (readString =="FORWARD"){
      rightMotor.run (FORWARD);
      leftMotor.run (FORWARD);
    }
    
    if (readString =="BACK"){
      rightMotor.run (BACKWARD);
      leftMotor.run (BACKWARD);
    }
    
    if (readString =="LEFT"){
      rightMotor.run (FORWARD);
      leftMotor.run (BACKWARD);
    }
    
    if (readString =="RIGHT"){
      rightMotor.run (BACKWARD);
      leftMotor.run (FORWARD);
    }
    
    if (readString =="STOP"){
      rightMotor.run (RELEASE);
      leftMotor.run (RELEASE);
    }

    if (readString == "GRAB"){
      servo1.write(90);
    }

    if (readString == "DROP"){
      servo1.write(175);
    }

    readString="";
  }
}
