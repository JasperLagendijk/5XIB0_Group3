#include <Arduino.h>
#include <Servo.h>
#include "logic_base.h"
#include "robot_base.h"

#define ANGLE_CHANGE 10
#define MAX_DISTANCE 20
Servo servo_ultrasound;


long measure_US(int angle) {
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

void findObstacleUS(double *x, double *y, double *phi, coords* head) {
  Serial.println("Entered function");
  //Initialize values
  long distance;
  int t;
  int angle = 90;
  int angle_prev = 85; 
  int temp=ANGLE_CHANGE;
  int store_max_angle = 0;
  int store_min_angle = 0; 

  for(int k = 0; k < 18; k++){ //Search until mountain is found 
  //int Dang_angle = 0;
  delay(500);
  angle = 180-k*ANGLE_CHANGE;
  distance = measure_US(angle);
  if (distance < MAX_DISTANCE) { //Object found find edges
    Serial.println("Object detected...");
    int left_distance = distance;
    int right_distance = distance;
    int left_angle = angle;
    int right_angle = angle;
    int angle_change;
    for(int i =1; i < 3; i++) { //Determine left edge angle
      delay(500);
      angle_change = 6/i;
      if (abs(left_distance-distance) <= 5) left_angle = left_angle+angle_change; 
      else left_angle = left_angle-angle_change;
      left_distance = measure_US(left_angle);
    }
    while (abs(right_distance-distance) <= 5) {
      delay(500);
      right_distance = measure_US(right_angle);
      right_angle = right_angle-ANGLE_CHANGE;
    }
    Serial.println("Exit sweep");
    for(int l =0; l < 3; l++) { //Determine right edge angle
      delay(500);
      angle_change = 6/l;
      
      if (abs(right_distance-distance) <= 5) right_angle = right_angle+angle_change; 
      else right_angle = right_angle-angle_change;
      right_distance = measure_US(right_angle);
    }


    
  }
    
  
  }
}
  /*if (distance < MAX_DISTANCE){
  Dang_angle=angle;
  Serial.println(Dang_angle);
  Serial.println(distance);
  delay(1000);
  }
  else{
  servo_ultrasound.attach(ultrasoundServo);
  }

  delay(500);
 
  if (angle>=180){
    temp = -ANGLE_CHANGE;
  }
  else if (angle <=0){
    temp = ANGLE_CHANGE;
  }
    angle +=temp ;
  } 
}*/


int detectLineIR() {
  
  
  
}
