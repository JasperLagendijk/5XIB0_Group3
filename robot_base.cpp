#include <Servo.h>

Servo servo_left;
Servo servo_right;
Servo servo_grabber;

servo_left.attach(servoLeft);
servo_right.attach(servoRight);
servo_grabber.attach(grabberServo);


void forward() {
  servo_left.writeMicroseconds(1600);
  servo_right.writeMicroseconds(1400); 
}

void backward() {
  servo_left.writeMicroseconds(1400);
  servo_right.writeMicroseconds(1600);
}

void brake() {
  servo_left.writeMicroseconds(1500);
  servo_right.writeMicroseconds(1500);
}


void turn(double psi, double * phi) {
  double distance;// = ((psi-)/2)*WIDTHROBOT;
   if(psi >= PI) {
    servo_left.writeMicroseconds(1400);
    servo_right.writeMicroseconds(1400);
    distance = ((psi-PI)/2)*0.104; //WIDTHROBOT;
  } else {
    servo_left.writeMicroseconds(1600);
    servo_right.writeMicroseconds(1600);
    distance = ((psi)/2)*0.104; //WIDTHROBOT;
  }
  
  
  Serial.println("Test 2.0");
  int left, right;
  double circumference = 0.21;
  int prev_left=left;
  int prev_right=right;
  double rotations = 0;
  double rotations_l = 0;
  double rotations_r = 0;
  double expectedRotations = distance/circumference;
  Serial.println(expectedRotations);
  Serial.println(rotations);
 
  

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

void moveTo(double x_start, double y_start, double x_end, double y_end) {
  //x_start < x_end
  double x_current = x_start;
  double y_current = y_start;
  double distance = x_end-x_start; 
  int left, right;
  double circumference = 0.21;
  int prev_left=left;
  int prev_right=right;
  double rotations = 0;
  double rotations_l = 0;
  double rotations_r = 0;
  double expectedRotations = distance/circumference;// / CIRCUMFERENCE;
  Serial.println(distance);
  Serial.println(CIRCUMFERENCE);
  Serial.println(expectedRotations);
  
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
      Serial.println(rotations);
  }
  brake(&dir);  
}
