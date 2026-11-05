#ifndef CAR_OBSERVER_H
#define CAR_OBSERVER_H
/* Core 0 only; never directly controls motors. Both peripherals are optional. */
void car_observer_init(void);
void car_observer_poll(void);
#endif
