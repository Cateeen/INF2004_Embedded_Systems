#include "obstacle_detection_task.h"

#include "obs_port.h"
#include "obs_scan.h"
#include "vehicle_messages.h"
#include "vehicle_system.h"

#include <stdio.h>
#include <string.h>

static uint32_t s_command_sequence;

bool obstacle_detection_request_scan(void)
{
    obstacle_command_message_t message;
    INT result;

    memset(&message, 0, sizeof(message));
    message.header.type = VEHICLE_MESSAGE_OBSTACLE_COMMAND;
    message.header.version = VEHICLE_MESSAGE_VERSION;
    message.header.sequence = ++s_command_sequence;
    message.header.timestamp_ms = obs_port_ms();
    message.command = VEHICLE_OBSTACLE_SCAN_REQUEST;
    result = tk_snd_mbf(g_mbf_obstacle_commands, &message,
                        (INT)sizeof(message), TMO_POL);
    if (result != E_OK) {
        printf("obstacle: scan request send failed (%d)\n", (int)result);
        return false;
    }
    return true;
}

void obstacle_detection_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    s_fault_sequence = 0U;
    s_command_sequence = 0U;

    obs_init();
    for (;;) {
        obs_step();
    }
}
