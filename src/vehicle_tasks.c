#include "vehicle_tasks.h"
#include "line_barcode_task.h"
#include "imu_task.h"
#include "obstacle_detection_task.h"
#include "motion_control_task.h"
#include "vehicle_controller_task.h"
#include "vehicle_odometry_task.h"
#include "wifi_service_task.h"

#define VEHICLE_TASK_ATTRIBUTE       (TA_HLNG | TA_RNG0)

#define LINE_BARCODE_TASK_PRIORITY   8
#define IMU_TASK_PRIORITY            6
#define OBSTACLE_TASK_PRIORITY       5
#define MOTION_TASK_PRIORITY         3
#define ODOMETRY_TASK_PRIORITY       7
#define CONTROLLER_TASK_PRIORITY     4
#define WIFI_TASK_PRIORITY           10

#define LINE_BARCODE_STACK_SIZE      2048U
#define IMU_STACK_SIZE               2048U
#define OBSTACLE_STACK_SIZE          2048U
#define MOTION_STACK_SIZE            2048U
#define ODOMETRY_STACK_SIZE          2048U
#define CONTROLLER_STACK_SIZE        3072U
#define WIFI_STACK_SIZE              4096U

typedef void (*vehicle_task_entry_t)(INT stacd, void *exinf);

ID g_tsk_line_barcode;
ID g_tsk_imu;
ID g_tsk_obstacle_detection;
ID g_tsk_motion_control;
ID g_tsk_vehicle_controller;
ID g_tsk_wifi_service;

typedef struct {
    ID *id;
    vehicle_task_entry_t entry;
    INT priority;
    INT stack_size;
} vehicle_task_definition_t;

static ER create_task(const vehicle_task_definition_t *definition)
{
    T_CTSK configuration;

    if ((definition == NULL) ||
        (definition->id == NULL) ||
        (definition->entry == NULL) ||
        (definition->priority <= 0) ||
        (definition->stack_size <= 0)) {
        return E_PAR;
    }

    configuration.exinf = NULL;
    configuration.tskatr = VEHICLE_TASK_ATTRIBUTE;
    configuration.task = definition->entry;
    configuration.itskpri = definition->priority;
    configuration.stksz = definition->stack_size;

    *(definition->id) = tk_cre_tsk(&configuration);
    if (*(definition->id) < 0) {
        return *(definition->id);
    }

    return E_OK;
}

static ER create_all_tasks(void)
{
    static const vehicle_task_definition_t definitions[] = {
        { &g_tsk_line_barcode, line_barcode_task, LINE_BARCODE_TASK_PRIORITY,
          (INT)LINE_BARCODE_STACK_SIZE },
        { &g_tsk_imu, imu_task, IMU_TASK_PRIORITY,
          (INT)IMU_STACK_SIZE },
        { &g_tsk_obstacle_detection, obstacle_detection_task,
          OBSTACLE_TASK_PRIORITY, (INT)OBSTACLE_STACK_SIZE },
        { &g_tsk_motion_control, motion_control_task, MOTION_TASK_PRIORITY,
          (INT)MOTION_STACK_SIZE },
        { &g_tsk_odometry, vehicle_odometry_task, ODOMETRY_TASK_PRIORITY,
          (INT)ODOMETRY_STACK_SIZE },
        { &g_tsk_vehicle_controller, vehicle_controller_task,
          CONTROLLER_TASK_PRIORITY, (INT)CONTROLLER_STACK_SIZE },
        { &g_tsk_wifi_service, wifi_service_task, WIFI_TASK_PRIORITY,
          (INT)WIFI_STACK_SIZE }
    };
    UINT index;
    ER error;

    for (index = 0U; index < (UINT)(sizeof(definitions) / sizeof(definitions[0]));
         index++) {
        error = create_task(&definitions[index]);
        if (error != E_OK) {
            return error;
        }
    }

    return E_OK;
}

static ER start_all_tasks(void)
{
    const ID task_ids[] = {
        g_tsk_line_barcode,
        g_tsk_imu,
        g_tsk_obstacle_detection,
        g_tsk_motion_control,
        g_tsk_odometry,
        g_tsk_vehicle_controller,
        g_tsk_wifi_service
    };
    UINT index;
    ER error;

    for (index = 0U; index < (UINT)(sizeof(task_ids) / sizeof(task_ids[0]));
         index++) {
        error = tk_sta_tsk(task_ids[index], 0);
        if (error != E_OK) {
            return error;
        }
    }

    return E_OK;
}

ER vehicle_tasks_create_and_start(void)
{
    ER error = create_all_tasks();

    if (error != E_OK) {
        return error;
    }

    return start_all_tasks();
}
