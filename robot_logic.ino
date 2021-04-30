//#include <Servo.h>
#include "robot_base.h"

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
  Servo servo_left;
  Servo servo_right;
  Servo servo_grabber;
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  servo_grabber.attach(grabberServo);
  
}

void loop () {

}
