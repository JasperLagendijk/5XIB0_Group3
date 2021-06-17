#include <Servo.h>
#include <Arduino.h>
#include <math.h>
#include "logic_base.h"
#include "sensor_base.h"

Servo servo_left;
Servo servo_right;
Servo servo_grabber;

#define RX_ZigBee 0
#define TX_ZigBee 1
//Digital pins 2-6 unused
#define encoderLeft 7
#define encoderRight 8
#define ultrasoundSensor 9 //PWM
#define grabberServo 10    //PWM
#define ultrasoundServo 11 //PWM
#define servoLeft 12
#define servoRight 13

#define grabberPin0 4
#define grabberPin1 5

#define CIRCUMFERENCE 2 * 0.034 * PI // In meters
#define WIDTHROBOT 0.104             // In meters

static double *glbl_phi = NULL;

void turn(double psi, double *phi);
void check_grabber_sensors(int *t2, double *angle_correction_container);

int cap(int input, int maximum)
{
  int output;
  if (abs(input) > maximum)
    output = input / abs(input) * maximum;
  else
    output = input;
  return output;
}

void brake()
{
  servo_grabber.attach(grabberServo);
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  servo_left.writeMicroseconds(1500);
  servo_right.writeMicroseconds(1500);
  servo_left.detach();
  servo_right.detach();
}

int bit_find(int val, int bit_find)
{
  return val & bit_find ? 1 : 0;
}

int grabber_pin_left = 0;
int grabber_pin_right = 0;

void read_servo_grabber_sensors(void)
{
  analogRead(grabberPin0);
  delay(10);
  grabber_pin_right = 15 * analogRead(grabberPin0);
  analogRead(grabberPin1);
  delay(10);
  grabber_pin_left = 10 * analogRead(grabberPin1);

  // // Logger for grabber IRs
  // Serial.print("grabber_pin_left:");
  // Serial.print(grabber_pin_left);
  // Serial.print(", ");

  // Serial.print("grabber_pin_right:");
  // Serial.print(grabber_pin_right);
  // Serial.println(", ");

  // Serial.print("Left:");
  // Serial.print((is_obst(grb_sens_left)));
  // Serial.print(", ");
  // Serial.print("Right:");
  // Serial.println((is_obst(grb_sens_right)));
  // Serial.print(", ");
  // Serial.print("t2:");
  // Serial.println(t2);
}

typedef enum
{
  grb_sens_left,
  grb_sens_right,
} sensor_name_e;

#define LEFT_IR_GRAB_TH 4500
#define RIGHT_IR_GRAB_TH 7500

int is_obst(sensor_name_e sensor)
{
  switch (sensor)
  {
  case grb_sens_left:
    return grabber_pin_left >= LEFT_IR_GRAB_TH ? 1 : 0;
  case grb_sens_right:
    return grabber_pin_right >= RIGHT_IR_GRAB_TH ? 1 : 0;
  }
}

void reatach_servos(void)
{
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  servo_grabber.attach(grabberServo);
}

double rads(double deg)
{
  return deg / 180 * PI;
}

void custom_turn(double psi, double *phi)
{
  turn(psi, phi);
}

#define deg_change 7
#define deg_gain 1.1

double drive(double distance, int dir)
{
  servo_grabber.attach(grabberServo);
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  int t2 = 0;
  double angle_correction = 0;
  int left, right;
  int mod_left = 0;
  int mod_right = 0;
  double circumference = 0.21;
  int prev_left = left;
  int prev_right = right;
  double rotations = 0;
  double rotations_l = 0;
  double rotations_r = 0;
  double expectedRotations = distance / circumference; // / CIRCUMFERENCE;
  int offset_power = 9;
  servo_grabber.write(180);
  int t = millis();
  double time_left = 0;
  while (rotations < expectedRotations)
  {
    if (dir)
    { // Drive forward
      servo_left.writeMicroseconds(1600 + mod_left);
      servo_right.writeMicroseconds(1400 + mod_right);
    }
    else
    { // Drive backward
      servo_left.writeMicroseconds(1400 + mod_left);
      servo_right.writeMicroseconds(1600 + mod_right);
    }

    left = digitalRead(encoderLeft);
    right = digitalRead(encoderRight);
    if (left != prev_left)
    { //1/8th of rotation made left
      rotations_l += (0.0625);
    }
    if (right != prev_right)
    { //1/8th of rotation made left
      rotations_r += 0.0625;
    }

    if (rotations_l > rotations_r && millis() - t > 50)
    { //If left wheel faster -> slow down left, speed up right
      if (dir)
      {
        mod_left -= offset_power;
        mod_right -= offset_power;
      }
      else
      {
        mod_left += offset_power;
        mod_right += offset_power;
      }
      t = millis();
    }
    else if (rotations_r > rotations_l && millis() - t > 50)
    {
      // If right wheel faster -> speed up left, slow down right
      if (dir)
      {
        mod_left += offset_power;
        mod_right += offset_power;
      }
      else
      {
        mod_left -= offset_power;
        mod_right -= offset_power;
      }
      t = millis();
    }
    mod_left = cap(mod_left, 38);
    mod_right = cap(mod_right, 38);
    rotations = (rotations_l + rotations_r) / 2;
    prev_left = left;
    prev_right = right;
    distance = rotations * circumference;
    long ultra = measure_US(90);

    // if (ultra<20){
    //   brake();
    //   servo_left.detach();
    //   servo_right.detach();
    //   return distance;
    // }

    // * Check the sensor array
    // * Kills the robot movement
    // * Does a 180
    // * Returns the distance to the main controller
    int sensors = sensorArray(sensor_array_check_mode_road);
    if (bit_find(sensors, 8) || bit_find(sensors, 4) || bit_find(sensors, 2) || bit_find(sensors, 1))
    {
      brake();
      turn(glbl_phi + rads(180), glbl_phi);
      return distance;
    }

    // * Check for LAB
    // TODO : Implement the thresholds that determine the LAB emptyness
    // sensors = sensorArray(sensor_array_check_mode_empty);
    // if(bit_find(sensors,8) || bit_find(sensors,4) || bit_find(sensors,2) || bit_find(sensors,1)){
    //   // Kills robot movement
    //   // Drops block
    //   brake();
    //   servo_grabber.attach(grabberServo);
    //   servo_grabber.write(180);
    //   t2 = millis();
    //   return distance;
    // }

    // * Logger for sensor array IRs
    // Serial.print(bit_find(sensors,8)); Serial.print(bit_find(sensors,4)); Serial.print(bit_find(sensors,2)); Serial.println(bit_find(sensors,1)); Serial.println("Finished printing");

    check_grabber_sensors(&t2, &angle_correction);

    // Time logger
    // Serial.print("t2:");Serial.print(t2);Serial.print("' ");Serial.print("time_left:");Serial.print(time_left);Serial.println("' ");time_left = millis() - t2;
    if ((time_left >= 300) && t2 > 0)
    {
      servo_grabber.attach(grabberServo);
      servo_grabber.write(180);
      t2 = 0;
      // * Trying to return to the same direction used before entering teh grabber maneuvers
      Serial.println("Returning to original angle");
      turn(angle_correction, glbl_phi);
      // Serial.println("Closing grabber");
      reatach_servos();
      // delay(1000);
    }
    // Logger for time passed after the grabber openned
    // Serial.print("t:");
    // Serial.println(time_left);
  }
  brake();
  servo_left.detach();
  servo_right.detach();
  return distance;
}

void check_grabber_sensors(int *t2, double *angle_correction_container)
{
  int state_changed = 0;
  read_servo_grabber_sensors();
  if ((is_obst(grb_sens_left) || is_obst(grb_sens_right)) && (*t2 == 0))
  {
    *angle_correction_container = *glbl_phi;
    Serial.println("Grabber sensor activated");
    brake();
    // servo_grabber.write(100);
    read_servo_grabber_sensors();
    if (is_obst(grb_sens_left))
    {
      custom_turn(*glbl_phi + rads(2 * deg_change), glbl_phi);
      delay(600);
      // Serial.println("Found left 1");
      state_changed = 1;
    }
    else if (is_obst(grb_sens_right))
    {
      // Serial.println("Found right 1");
      state_changed = 1;
      custom_turn(*glbl_phi - rads(2 * deg_change), glbl_phi);
      delay(600);
    }
    else
    {
      // Serial.println("Found nthng 2");
      state_changed = 1;
      custom_turn(*glbl_phi + rads(deg_gain * deg_change), glbl_phi);
      delay(600);
      read_servo_grabber_sensors();
      if (is_obst(grb_sens_left))
      {
        custom_turn(*glbl_phi + rads(deg_change), glbl_phi);
        delay(600);
        // Serial.println("Found left 2");
        state_changed = 1;
      }
      else if (is_obst(grb_sens_right))
      {
        custom_turn(*glbl_phi - rads(deg_change), glbl_phi);
        delay(600);
        // Serial.println("Found right 2");
        state_changed = 1;
      }
      else
      {
        // Serial.println("Found nthng 3");
        state_changed = 1;
        custom_turn(*glbl_phi - rads(2 * deg_gain * deg_change), glbl_phi);
        delay(600);
        read_servo_grabber_sensors();
        if (is_obst(grb_sens_left))
        {
          custom_turn(*glbl_phi + rads(deg_change), glbl_phi);
          delay(600);
          // Serial.println("Found left 3");
          state_changed = 1;
        }
        else if (is_obst(grb_sens_right))
        {
          custom_turn(*glbl_phi - rads(deg_change), glbl_phi);
          delay(600);
          // Serial.println("Found right 3");
          state_changed = 1;
        }
        else
        {
          custom_turn(*glbl_phi + rads(deg_gain * deg_change), glbl_phi);
          delay(600);
          // Serial.println("Found nothing | Back to original degree");
          state_changed = 1;
        }
      }
    }

    if (state_changed)
    {
      *t2 = millis(); //close the grabber
      state_changed = 0;
    }

    servo_grabber.attach(grabberServo);
    servo_grabber.write(100);
    reatach_servos();
    // make a more accurate estimation of the rock's position and grab it
    // delay(2000);
  }
}

void turn(double psi, double *phi)
{ //Turn to psi, from starting rotation phi, update phi
  // Serial.print("new:");
  // Serial.print(psi);
  // Serial.print(", cur:");
  // Serial.println(*phi);
  brake();
  servo_grabber.attach(grabberServo);
  // servo_grabber.write(10);
  servo_left.attach(servoLeft);
  servo_right.attach(servoRight);
  double distance = 0; // = ((psi-)/2)*WIDTHROBOT;
  if (psi - *phi >= 0)
  { // Turn left
    servo_left.writeMicroseconds(1400);
    servo_right.writeMicroseconds(1400);
    distance = ((abs(psi - *phi)) / 2) * 0.104; //WIDTHROBOT;
  }
  else
  { //Turn right
    servo_left.writeMicroseconds(1600);
    servo_right.writeMicroseconds(1600);
    distance = ((abs(psi - *phi)) / 2) * 0.104; //WIDTHROBOT;
  }

  int left = 0, right = 0;
  double circumference = 0.21;
  int prev_left = left;
  int prev_right = right;
  double rotations = 0;
  double rotations_l = 0;
  double rotations_r = 0;
  double expectedRotations = distance / circumference;
  //Serial.println(psi-*phi);
  //Serial.println(distance);
  while (rotations < expectedRotations)
  {
    // Serial.print("rotations:");
    // Serial.print(rotations);
    // Serial.print(", ");
    // Serial.print("expectedRotations:");
    // Serial.print(expectedRotations);
    // Serial.println(", ");
    left = digitalRead(encoderLeft);
    right = digitalRead(encoderRight);
    if (left != prev_left)
    { //1/8th of rotation made left
      rotations_l += (0.0625);
    }
    if (right != prev_right)
    { //1/8th of rotation made left
      rotations_r += 0.0625;
    }
    rotations = (rotations_l + rotations_r) / 2;
    prev_left = left;
    prev_right = right;
  }
  brake();
  *phi = psi;
  servo_grabber.detach();
  servo_left.detach();
  servo_right.detach();
}

int scanner_on = false;

int moveTo(double *x_start, double *y_start, double x_end, double y_end, double *phi)
{ // Function moves robot to location (x_end, y_end), returns 0 if succesfull, -1 if blocked
  double current_x = *x_start;
  double current_y = *y_start;
  double dx = x_end - *x_start;
  double dy = y_end - *y_start;
  double distance = sqrt(pow(dx, 2) + pow(dy, 2));
  double psi = atan2(dy, dx);
  //if(psi < 0) psi += 6.28;
  turn(psi, phi);
  *phi = psi;
  glbl_phi = phi;
  distance = drive(distance, 1);
  current_x += distance * cos(psi);
  current_y += distance * sin(psi);
  *x_start = current_x;
  *y_start = current_y;
  if (abs(current_x - x_end) <= 0.05 && abs(current_y - y_end) <= 0.05)
  { // Destination reached
    return 0;
  }
  else
    return sensorArray(sensor_array_check_mode_road); // Otherwise: obstacle encountered
}

void drivePath(node *top, double *x_start, double *y_start, double *phi)
{
  while (top->next != NULL)
  {
    *x_start = top->x;
    *y_start = top->y;
    double x_end = top->next->x;
    double y_end = top->next->y;
    int sensors = moveTo(x_start, y_start, x_end, y_end, phi);
    // if(sensor > 0) {
    //   if (*phi >= -0.7854 && *phi <= 0.7854 )  addObstacle(head, *x_start-0.05, *y_start-0.05, *x_start+0.1, *y_start+0.05);  //Driving in the positive x direction
    //   else if (*phi >= 0.7854 && *phi <= 3.9269)  addObstacle(head, *x_start-0.05, *y_start-0.05, *x_start+0.05, *y_start+0.1);                //Driving in the positive y direction
    //   else if (*phi <= -0.7854 && *phi >= -3.9269) addObstacle(head, *x_start-0.05, *y_start-0.1, *x_start+0.05, *y_start+0.05);               //Driving in the negative y direction
    //   else if (*phi <= -3.9269 && *phi >= 3.9269) addObstacle(head, *x_start-0.1, *y_start-0.05, *x_start+0.05, *y_start+0.05);                //Driving in the negative x direction
    // }
    popNode(&top);
  }
}
