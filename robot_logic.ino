#include "robot_base.h"
#include "logic_base.h"

double phi = 1.5708;
double x = 0;
double y = 0;

node * start = NULL;
coords intersect_struct;
head = (coords *) malloc(sizeof(coords));
head->x_min = 5;
head->x_max = 6;
head->y_min = -3;
head->y_max = 3;
if (head == NULL) {
  return -1;
}

start = (node *) malloc(sizeof(coords));
start->x = 0;
start->y = 0;

addNode(start, 10, 0);

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
}

void loop () {
  
}
