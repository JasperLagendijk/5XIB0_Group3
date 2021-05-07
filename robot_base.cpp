#include <Servo.h>
#include <Arduino.h>
#include <math.h>

Servo servo_left;
Servo servo_right;
Servo servo_grabber;

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

#define CIRCUMFERENCE 2*0.034*PI // In meters
#define WIDTHROBOT 0.104 // In meters


void forward() {
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  servo_left.writeMicroseconds(1600);
  servo_right.writeMicroseconds(1400);
}

void backward() {
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  servo_left.writeMicroseconds(1400);
  servo_right.writeMicroseconds(1600);
}

void brake() {
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  servo_left.writeMicroseconds(1500);
  servo_right.writeMicroseconds(1500);
}


void turn(double psi, double * phi) {
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  double distance;// = ((psi-)/2)*WIDTHROBOT;
   if(psi >= 3.141) {
    servo_left.writeMicroseconds(1400);
    servo_right.writeMicroseconds(1400);
    distance = ((psi-3.141)/2)*0.104; //WIDTHROBOT;
  } else {
    servo_left.writeMicroseconds(1600);
    servo_right.writeMicroseconds(1600);
    distance = ((psi)/2)*0.104; //WIDTHROBOT;
  }


  //Serial.println("Test 2.0");
  int left, right;
  double circumference = 0.21;
  int prev_left=left;
  int prev_right=right;
  double rotations = 0;
  double rotations_l = 0;
  double rotations_r = 0;
  double expectedRotations = distance/circumference;
  ///Serial.println(expectedRotations);
  //Serial.println(rotations);



    while(rotations < expectedRotations) {
      left = digitalRead(encoderLeft);
      right = digitalRead(encoderRight);
      if(left != prev_left) { //1/8th of rotation made left
        rotations_l+= (0.0625);
        //Serial.println(rotations_l);
      }
      if(right != prev_right) { //1/8th of rotation made left
        rotations_r += 0.0625;
      }
      //if (abs(rotations_r - rotations_l) > 1) {
        //ERROR
      //} else {
        rotations = (rotations_l+rotations_r)/2;
      //}
      prev_left = left;
      prev_right = right;
      Serial.println(rotations);

    }
    brake();
}

void moveTo(double x_start, double y_start, double x_end, double y_end, double * phi) {
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  //x_start < x_end
  double x_current = x_start;
  double y_current = y_start;
  double dx = x_end-x_start;
  double dy = y_end-y_start;
  double distance = sqrt(pow(dx, 2)+pow(dy, 2));
  int left, right;
  double circumference = 0.21;
  int prev_left=left;
  int prev_right=right;
  double rotations = 0;
  double rotations_l = 0;
  double rotations_r = 0;
  double expectedRotations = distance/circumference;// / CIRCUMFERENCE;
  //Serial.println(distance);
  //Serial.println(CIRCUMFERENCE);
  //Serial.println(expectedRotations);
  double psi = atan2(dy, dx);
  Serial.println(psi);
  turn(psi, phi);
  forward();

  while(rotations < expectedRotations) {
      left = digitalRead(encoderLeft);
      right = digitalRead(encoderRight);
      if(left != prev_left) { //1/8th of rotation made left
        rotations_l+= (0.0625);
        //Serial.println(rotations_l);
      }
      if(right != prev_right) { //1/8th of rotation made left
        rotations_r += 0.0625;
      }
      //if (abs(rotations_r - rotations_l) > 1) {
        //ERROR
      //} else {
        rotations = (rotations_l+rotations_r)/2;
      //}
      prev_left = left;
      prev_right = right;
      //Serial.println(rotations);
  }
  brake();
}
