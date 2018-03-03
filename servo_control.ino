`/* Before you can use the Motor shield, you must install the AF_Motor Arduino library
https://github.com/adafruit/Adafruit-Motor-Shield-library 
Install and Use Arduino Libraries -- https://youtu.be/Es8D1q_b-c4 */
#include <AFMotor.h>

/*include Servo motor library*/
#include <Servo.h>

/* Create the AF_DCMotor object with AF_DCMotor(motor#, frequency)
The first is which port the motor is connected to, 1, 2, 3 or 4.
frequency is how fast the speed controlling signal is. 
For motors 1 and 2 you can choose MOTOR12_64KHZ, MOTOR12_8KHZ, MOTOR12_2KHZ, orMOTOR12_1KHZ. 
A high speed like 64KHz wont be audible but a low speed like 1KHz will use less power. 
Motors 3 & 4 are only possible to run at 1KHz and will ignore any setting given */
AF_DCMotor leftMotor(1, MOTOR12_8KHZ);
AF_DCMotor rightMotor(2, MOTOR12_8KHZ);

/*Define Servo Name*/
Servo servo1;

/*Set the speed of the motor
The speed ranges from 0 (stopped) to 255 (full speed) 
You can set the speed whenever you want.*/
#define rightSpeed 160
#define leftSpeed 110

String readString;

void setup() {
  Serial.begin(9600);

  /*Digital 10 pin controls the Servo#1 input
  Digital 9 pin controls the Servo#2 input */
  servo1.attach(10);
  
  /*Servo motor starting position*/
  servo1.write(175);

  /*you can set the speed of the motor using setSpeed(speed)*/
  rightMotor.setSpeed(rightSpeed);
  leftMotor.setSpeed(leftSpeed);
}

void loop() {
  while(Serial.available()){
    delay(50);
    char c=Serial.read();
    readString+=c;
  }
  if(readString.length()>0){
    Serial.println(readString);

    /*To run the motor, call run(direction) where direction is FORWARD, BACKWARD or RELEASE.*/
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
