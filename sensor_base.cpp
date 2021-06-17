#include <Arduino.h>
#include <Servo.h>
#include "logic_base.h"
#include "robot_base.h"

#define analogPin0 0
#define analogPin1 1
#define analogPin2 2
#define analogPin3 3

#define ANGLE_CHANGE 10
#define MAX_DISTANCE 20
Servo servo_ultrasound;

int availableMemory()
{
  int size = 1024; // Use 2048 with ATmega328
  byte *buf;

  while ((buf = (byte *)malloc(--size)) == NULL)
    ;

  free(buf);

  return size;
}

long measure_US(int angle)
{
  servo_ultrasound.attach(ultrasoundServo);
  long duration, distance;
  servo_ultrasound.write(angle);
  pinMode(ultrasoundSensor, OUTPUT);
  digitalWrite(ultrasoundSensor, LOW);
  delayMicroseconds(2);
  digitalWrite(ultrasoundSensor, HIGH);
  delayMicroseconds(5);
  digitalWrite(ultrasoundSensor, LOW);

  pinMode(ultrasoundSensor, INPUT);
  duration = pulseIn(ultrasoundSensor, HIGH);
  distance = duration / 29 / 2;
  return distance;
}

void findObstacleUS(double *x, double *y, double *phi, coords *head)
{
  long distance;
  int t;
  int angle = 90;
  int angle_prev = 85;
  int temp = ANGLE_CHANGE;
  int store_max_angle = 0;
  int store_min_angle = 0;

  for (int k = 0; k < 18; k++)
  { //Search until mountain is found
    delay(500);
    angle = 180 - k * ANGLE_CHANGE;
    distance = measure_US(angle);
    if (distance < MAX_DISTANCE)
    { //Object found find edges
      int left_distance = distance;
      int right_distance = distance;
      int left_angle = angle;
      int right_angle = angle;
      int angle_change;
      for (int i = 1; i < 3; i++)
      { //Determine left edge angle
        delay(500);
        angle_change = 6 / i;
        if (abs(left_distance - distance) <= 5)
          left_angle = left_angle + angle_change;
        else
          left_angle = left_angle - angle_change;
        left_distance = measure_US(left_angle);
      }
      while (abs(right_distance - distance) <= 5)
      {
        delay(500);
        right_distance = measure_US(right_angle);
        right_angle = right_angle - ANGLE_CHANGE;
      }
      for (int l = 1; l < 3; l++)
      { //Determine right edge angle
        delay(500);
        angle_change = 6 / l;
        if (abs(right_distance - distance) <= 5)
          right_angle = right_angle + angle_change;
        else
          right_angle = right_angle - angle_change;
        right_distance = measure_US(right_angle);
      }
      //Adding obstacle
      double right_angle_rad = (90 - static_cast<double>(right_angle)) * (3.1415926 / 180); //Convert angle from radians to degrees and add compensation in relation to robot
      double left_angle_rad = (90 - static_cast<double>(left_angle)) * (3.1415926 / 180);   //Convert angle from radians to degrees and add compensation in relation to robot
      double dx_r = cos(*phi + right_angle_rad) * right_distance;
      double dy_r = sin(*phi + right_angle_rad) * right_distance;

      double dx_l = cos(*phi + left_angle_rad) * left_distance;
      double dy_l = sin(*phi + left_angle_rad) * left_distance;

      double x_obstacle = *x + ((dx_r + dx_l) / 2);
      double y_obstacle = *y + ((dy_r + dy_l) / 2);

      double dx = abs((1.5 * dx_r) + (0.5 * dx_l));
      double dy = abs((1.5 * dy_r) + (0.5 * dy_l));

      double x_min = x_obstacle - dx - 0.5;
      double y_min = y_obstacle - dy - 0.5;
      double x_max = x_obstacle + dx + 0.5;
      double y_max = y_obstacle + dy + 0.5;

      addObstacle(head, x_min, x_max, y_min, y_max);
    }
  }
}

int sensorArray(sensor_array_check_mode_e mode)
{
  int e = 0;
  int black = 140;

  analogRead(analogPin0);
  delay(10);
  int sensor_read_0 = (analogRead(analogPin0));
  delay(10);

  analogRead(analogPin1);
  delay(10);
  int sensor_read_1 = (analogRead(analogPin1));

  delay(10);

  analogRead(analogPin2);
  delay(10);
  int sensor_read_2 = (analogRead(analogPin2));

  delay(10);

  analogRead(analogPin3);
  delay(10);
  int sensor_read_3 = (analogRead(analogPin3));

  switch (mode)
  {
  case sensor_array_check_mode_empty:
    if (sensor_read_0 < black)
      e += 1; //DigitalValue0 = 1;
    if (sensor_read_1 < 350)
      e += 2; //DigitalValue1 = 1;
    if (sensor_read_2 < 140)
      e += 4; //DigitalValue2 = 1;
    if (sensor_read_3 < black)
      e += 8; // DigitalValue3 = 1;
    break;
  case sensor_array_check_mode_road:
    if (sensor_read_0 < black)
      e += 1; //DigitalValue0 = 1;
    if (sensor_read_1 < 350)
      e += 2; //DigitalValue1 = 1;
    if (sensor_read_2 < 140)
      e += 4; //DigitalValue2 = 1;
    if (sensor_read_3 < black)
      e += 8; // DigitalValue3 = 1;
    break;
  }

  // * Sensor array logger
  //  Serial.print("sensor_read_0:"); Serial.print(sensor_read_0); Serial.print(", "); Serial.print("sensor_read_1:"); Serial.print(sensor_read_1); Serial.print(", "); Serial.print("sensor_read_2:"); Serial.print(sensor_read_2); Serial.print(", "); Serial.print("sensor_read_3:"); Serial.println(sensor_read_3);
  return e;
}
