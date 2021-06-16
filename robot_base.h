#ifndef ROBOT_BASE
#define ROBOT_BASE
#include <Servo.h>
#include "logic_base.h"

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


double drive(double distance, int dir);

void brake();
void turn(double psi, double * phi);
void moveTo(double *x_start, double *y_start, double x_end, double y_end, double * phi);
void drivePath(node * top, double * x_start, double * y_start, double * phi, coords *head);

#endif
