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
double phi = 0;
double x = 0;
double y = 0;
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
  moveTo(&x, &y, 1, 1, &phi);
  //drive(1, 1);
  //int psi = 0.25*3.141*i;
  //turn(psi, &phi);
  //servo_left.writeMicroseconds(1600);
  //servo_right.writeMicroseconds(1400);
  Serial.println(phi);
  delay(2000);
  moveTo(&x, &y, 0, 0, &phi);
  Serial.println(phi);
  //brake();
  //servo_left.writeMicroseconds(1500);
  //servo_right.writeMicroseconds(1500);
}
