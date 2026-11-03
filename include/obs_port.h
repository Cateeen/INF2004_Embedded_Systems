/* obs_port.h - everything Buddy 5 needs from the board and RTOS. */
#ifndef OBS_PORT_H
#define OBS_PORT_H

#include <stdint.h>
#include <stdbool.h>
#include "obs_pack.h"

typedef enum {
    OBS_ECHO_OK = 0,        /* pulse measured                                   */
    OBS_ECHO_NO_OBJECT,     /* ECHO rose but nothing within range window        */
    OBS_ECHO_NO_RESPONSE    /* ECHO never rose: sensor/wiring problem ("no echo") */
} obs_echo_t;

/* Set up the hardware and kernel objects: TRIG/ECHO pins, ECHO edge interrupt, servo PWM, echo event flag. Call once before the task runs.*/
void     obs_port_init(void);

/* Return a millisecond timestamp (monotonic, 32 bit). */
uint32_t obs_port_ms(void);

/* Sleep the task for ms milliseconds using the RTOS, so other tasks keep running. Never a busy loop.*/
void     obs_port_delay_ms(uint32_t ms);                     /* RTOS delay, never a busy loop */

/* Command the servo to deg (0 = right, 90 = ahead, 180 = left). Returns immediately; the caller waits for the servo to arrive. */
void     obs_port_servo_deg(uint8_t deg);                    /* command only; caller waits    */
obs_echo_t obs_port_measure(uint16_t *pulse_us);             /* TRIG + ISR timestamps + timeout */

/* Receive one 1-byte command from the Vehicle Controller. Waits up to timeout_ms (0 = poll). Returns true if a command was read. */
bool     obs_port_cmd_recv(uint8_t *cmd, uint32_t timeout_ms);   /* from Vehicle Controller */
void     obs_port_msg_send(const obs_msg_t *m);                  /* to Vehicle Controller   */

/* Set the given Buddy 5 fault bits in the team fault word. Must be an atomic set (kernel event flag), never a read-modify-write. */
void     obs_port_fault_set(uint32_t obs_fault_bits);        /* atomic set in team fault word  */
void     obs_port_obstacle_flag(bool present);               /* flag read by Safety Monitor    */
bool     obs_port_line_detected(void);                       /* Buddy 3 line_status            */

#endif