//#include <Servo.h>
#include "robot_base.h"
/*
#define RX_ZigBee 0
#define TX_ZigBee 1
//Digital pins 2-6 unused
#define encoderLeft 7
#define encoderRight 8
#define ultrasoundSensor 9 //PWM
#define grabberServo 10 //PWM
#define ultrasoundServo 11 //PWM
#define servoLeft 12
#define servoRight 13
*/
double phi;
/*  Servo servo_left;
  Servo servo_right;
  Servo servo_grabber;
*/
void setup() {
  Serial.begin(9600);
  pinMode(RX_ZigBee, INPUT);
  pinMode(TX_ZigBee, OUTPUT);
  //Digital pins 2-6 unused
  pinMode(encoderLeft, INPUT);
  pinMode(encoderRight, INPUT);
  pinMode(ultrasoundSensor, INPUT);
  pinMode(grabberServo, OUTPUT);
  pinMode(ultrasoundServo, OUTPUT);
  
 
  //servo_grabber.attach(grabberServo);
  
}

void loop () {

  delay(1000);
  moveTo(0, 0, 2, 0, &phi);
  
  //servo_left.writeMicroseconds(1600);
  //servo_right.writeMicroseconds(1400);
  delay(2000);
  //brake();
  //servo_left.writeMicroseconds(1500);
  //servo_right.writeMicroseconds(1500);
}
