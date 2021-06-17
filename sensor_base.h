#ifndef SENSOR_BASE
#define SENSOR_BASE

typedef enum
{
  sensor_array_check_mode_empty = 0,
  sensor_array_check_mode_road = 1
} sensor_array_check_mode_e;

void findObstacleUS(double *x, double *y, double *phi, coords* head);
int sensorArray(sensor_array_check_mode_e mode);
int availableMemory();
long measure_US(int angle);


















#endif
