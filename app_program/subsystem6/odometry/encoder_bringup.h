#ifndef ENCODER_BRINGUP_H
#define ENCODER_BRINGUP_H

#include <tk/tkernel.h>

#if APP_ODOMETRY_TEST || APP_SENSOR_TEST == 5
ER encoder_bringup_start(void);
#endif

#endif
