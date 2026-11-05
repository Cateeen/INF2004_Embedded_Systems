#ifndef CAR_MEASUREMENTS_H
#define CAR_MEASUREMENTS_H
#include <stdbool.h>
/* Synchronized Core 1 encoder distance -> Core 0 IMU observer. */
void car_measurements_init(void);
void car_measurements_publish_distance(float meters);
bool car_measurements_read_distance(float *meters);
#endif
