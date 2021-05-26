#include "robot_base.h"
#include "logic_base.h"

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
  head->x_min = 5;
  head->y_min = -3;
  head->x_max = 6;
  head->y_max = 3;
  head->next = NULL;
  head->prev = NULL;

  start = (node *) malloc(sizeof(node));
  start->x = 0;
  start->y = 0;
  addNode(start, 10, 0);
  intersect_struct = intersection(head, start);
  intersect_struct.x_max = 10;
  Serial.println(intersect_struct.x_min);
}

void loop () {
  Serial.println(phi);
  moveTo(&x, &y, 0.2, 0.2, &phi);
  Serial.println(phi);
  delay(1000);
  moveTo(&x, &y, 0.8, 0, &phi);
  Serial.println(phi);
  delay(1000);
  moveTo(&x, &y, 0, 0, &phi);
  Serial.println(phi);
  delay(1000);
}
