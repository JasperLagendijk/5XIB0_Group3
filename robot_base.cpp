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

int cap(int input, int maximum) {
  int output;
  if (abs(input) > maximum) output = input/abs(input)*maximum;
  else output = input;
  return output;
  
}

void brake() {
  servo_grabber.attach(grabberServo);
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  servo_left.writeMicroseconds(1500);
  servo_right.writeMicroseconds(1500);
  servo_left.detach();
  servo_right.detach();
}

double drive(double distance, int dir)
{
  servo_grabber.attach(grabberServo);
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);

  int left, right;
  int mod_left = 0;
  int mod_right = 0;
  double circumference = 0.21;
  int prev_left=left;
  int prev_right=right;
  double rotations = 0;
  double rotations_l = 0;
  double rotations_r = 0;
  double expectedRotations = distance/circumference;// / CIRCUMFERENCE;
  int offset_power = 10;
  servo_grabber.write(-45);
  int t = millis();
  while(rotations < expectedRotations) {
    if(dir) { // Drive forward
      servo_left.writeMicroseconds(1600+mod_left);
      servo_right.writeMicroseconds(1400+mod_right);
    }
    else {// Drive backward
      servo_left.writeMicroseconds(1400+mod_left);
      servo_right.writeMicroseconds(1600+mod_right);
    }

    left = digitalRead(encoderLeft);
    right = digitalRead(encoderRight);
    if(left != prev_left) { //1/8th of rotation made left
      rotations_l+= (0.0625);
      }
    if(right != prev_right) { //1/8th of rotation made left
      rotations_r += 0.0625;
    }
    
    if (rotations_l > rotations_r &&  millis()-t  > 1) { //If left wheel faster -> slow down left, speed up right
        //Serial.println("Rubber ducky");
        if(dir) {
          mod_left -= offset_power;
          mod_right -= offset_power;
        } else {
          mod_left += offset_power;
          mod_right += offset_power;
        }
        t = millis();
      } else if (rotations_r > rotations_l && millis()-t > 1) { // If right wheel faster -> speed up left, slow down right
        //Serial.println("Wooden ducky");
        if(dir) {
          mod_left += offset_power;
          mod_right += offset_power;
        } else {
          mod_left -= offset_power;
          mod_right -= offset_power;
        }
        t = millis();
      }
    mod_left = cap(mod_left, 40);
    mod_right = cap(mod_right, 40);
    rotations = (rotations_l+rotations_r)/2;
    prev_left = left;
    prev_right = right;
    if(0) { //If a cliff is detected, stop and return current distance travelled
      brake();
      servo_left.detach();
      servo_right.detach();
      return rotations*circumference;
    }
  }
  brake();
  servo_left.detach();
  servo_right.detach();
  return distance;
}


void turn(double psi, double * phi) {
  servo_grabber.attach(grabberServo);
  servo_grabber.write(20);
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  double distance;// = ((psi-)/2)*WIDTHROBOT;
   if(psi-*phi >= 3.141) {
    servo_left.writeMicroseconds(1400);
    servo_right.writeMicroseconds(1400);
    distance = ((psi-*phi-3.141)/2)*0.104; //WIDTHROBOT;
  } else {
    servo_left.writeMicroseconds(1600);
    servo_right.writeMicroseconds(1600);
    distance = ((psi-*phi)/2)*0.104; //WIDTHROBOT;
  }
  Serial.println(distance);
  int left, right;
  double circumference = 0.21;
  int prev_left=left;
  int prev_right=right;
  double rotations = 0;
  double rotations_l = 0;
  double rotations_r = 0;
  double expectedRotations = distance/circumference;

  while(rotations < expectedRotations) {
    left = digitalRead(encoderLeft);
    right = digitalRead(encoderRight);
    if(left != prev_left) { //1/8th of rotation made left
      rotations_l+= (0.0625);
    }
    if(right != prev_right) { //1/8th of rotation made left
      rotations_r += 0.0625;
    }
    rotations = (rotations_l+rotations_r)/2;
    prev_left = left;
    prev_right = right;
    //Serial.println(rotations);

    }
    brake();
    servo_grabber.detach();
    servo_left.detach();
    servo_right.detach();
  }

void moveTo(double x_start, double y_start, double x_end, double y_end, double * phi) {
  double current_x = x_start;
  double current_y = y_start;
  double dx = x_end-x_start;
  double dy = y_end-y_start;
  Serial.print("dx: ");
  Serial.print(dx);
  Serial.print(" dy: ");
  Serial.println(dy);
  double distance = sqrt(pow(dx, 2)+pow(dy, 2));
  double psi = atan2(dy, dx);
  if(psi < 0) psi += 6.28;
  Serial.print("Phi: ");
  Serial.print(*phi);
  Serial.print(" Psi: ");
  Serial.println(psi);
  turn(psi, phi);
  *phi = psi;
  distance = drive(distance, 1);
  current_x += distance*cos(psi);
  current_y += distance*sin(psi);
  if(current_x == x_end && current_y == y_end) { // Destination reached
    return;
  }
  // Otherwise: obstacle encountered


}
