#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "buddy3.h"
#include "motor_driver.h"
#include "pico/stdlib.h"

#define CONTROL_PERIOD_MS 10U
#define LOST_CONTINUE_MS  300U
#define FORWARD_PWM       25
#define CORRECTION_PWM    10

typedef enum
{
    LAST_FORWARD,
    LAST_LEFT,
    LAST_RIGHT
} last_motion_t;

static const char *position_name(uint8_t position)
{
    switch (position)
    {
        case POS_JUNCTION: return "JUNCTION";
        case POS_LEFT:     return "LEFT";
        case POS_RIGHT:    return "RIGHT";
        case POS_LOST:     return "LOST";
        default:           return "UNKNOWN";
    }
}

static void set_line_motion(uint8_t position, last_motion_t *last_motion)
{
    switch (position)
    {
        case POS_JUNCTION:
            *last_motion = LAST_FORWARD;
            motor_set_speed(MOTOR_1, FORWARD_PWM);
            motor_set_speed(MOTOR_2, -FORWARD_PWM);
            break;
        case POS_LEFT:
            *last_motion = LAST_LEFT;
            motor_set_speed(MOTOR_1, FORWARD_PWM - CORRECTION_PWM);
            motor_set_speed(MOTOR_2, -FORWARD_PWM);
            break;
        case POS_RIGHT:
            *last_motion = LAST_RIGHT;
            motor_set_speed(MOTOR_1, FORWARD_PWM);
            motor_set_speed(MOTOR_2, -(FORWARD_PWM - CORRECTION_PWM));
            break;
        default:
            motor_brake_all();
            break;
    }
}

static void repeat_last_motion(last_motion_t last_motion)
{
    switch (last_motion)
    {
        case LAST_FORWARD:
            motor_set_speed(MOTOR_1, FORWARD_PWM);
            motor_set_speed(MOTOR_2, -FORWARD_PWM);
            break;
        case LAST_LEFT:
            motor_set_speed(MOTOR_1, FORWARD_PWM - CORRECTION_PWM);
            motor_set_speed(MOTOR_2, -FORWARD_PWM);
            break;
        case LAST_RIGHT:
            motor_set_speed(MOTOR_1, FORWARD_PWM);
            motor_set_speed(MOTOR_2, -(FORWARD_PWM - CORRECTION_PWM));
            break;
        default:
            motor_brake_all();
            break;
    }
}

int main(void)
{
    ir_readings_t readings;
    last_motion_t last_motion = LAST_FORWARD;
    uint32_t lost_since_ms = 0U;
    bool line_was_lost = false;
    bool stopped_for_lost_line = false;

    stdio_init_all();
    sleep_ms(2000U);
    line_following_init();
    motor_driver_init();
    motor_clear_emergency_stop();

    if (motor_is_emergency_stopped())
    {
        printf("Failed to clear motor emergency stop.\n");
        return 1;
    }

    printf("Line-following differential-drive test started.\n");

    while (true)
    {
        uint32_t now_ms;
        uint8_t position;

        if (!line_following_read(&readings))
        {
            motor_emergency_stop();
            printf("IR sensor read failed; emergency stop.\n");
            break;
        }

        position = line_following_position(&readings);
        now_ms = to_ms_since_boot(get_absolute_time());

        printf("barcode=%u/%u center=%u/%u right=%u/%u position=%s\n",
               (unsigned)readings.barcode.raw, readings.barcode.black,
               (unsigned)readings.center.raw, readings.center.black,
               (unsigned)readings.right.raw, readings.right.black,
               position_name(position));

        if (position == POS_LOST)
        {
            if (!line_was_lost)
            {
                line_was_lost = true;
                lost_since_ms = now_ms;
            }
            else if (!stopped_for_lost_line &&
                     (now_ms - lost_since_ms) >= LOST_CONTINUE_MS)
            {
                motor_brake_all();
                stopped_for_lost_line = true;
                printf("Line still lost; stopping.\n");
            }
            else if (!stopped_for_lost_line)
            {
                repeat_last_motion(last_motion);
            }
        }
        else
        {
            line_was_lost = false;
            if (stopped_for_lost_line)
            {
                stopped_for_lost_line = false;
                printf("Line detected again; resuming.\n");
            }
            set_line_motion(position, &last_motion);
        }

        sleep_ms(CONTROL_PERIOD_MS);
    }

    return 0;
}
