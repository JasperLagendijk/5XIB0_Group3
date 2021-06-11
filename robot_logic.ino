#include "robot_base.h"
#include "logic_base.h"
#include "sensor_base.h"

double phi = 1.5708;
double x = 0;
double y = 0;

node * start = NULL;
coords intersect_struct;
coords * head = NULL;

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
  
  head = (coords *) malloc(sizeof(coords));
  head->x_min = 0.4;
  head->y_min = -0.2;
  head->x_max = 0.5;
  head->y_max = 0.3;
  head->next = NULL;
  head->prev = NULL;

  start = (node *) malloc(sizeof(node));
  start->x = 0;
  start->y = 0;
}

void loop () {
  //determinePath(&x, &y, &phi, head, 1, 0);
  //drivePath(start, &x, &y, &phi);
  //findObstacleUS (&x, &y, &phi, head);
  //delay(1000);
  Serial.println(digitalRead(encoderLeft));
  
}
