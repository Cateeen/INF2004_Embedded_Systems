
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "pid_controller.h"

static int approximately_equal(float a, float b)
{
    return fabsf(a - b) < 0.001f;
}

int main(void)
{
    pid_controller_t pid;

    pid_init(&pid, 1.0f, 0.0f, 0.0f, -100.0f, 100.0f);

    float output = pid_update(&pid, 100.0f, 70.0f, 0.01f);
    assert(approximately_equal(output, 30.0f));
    printf("PASS: Proportional control\n");

    output = pid_update(&pid, 300.0f, 0.0f, 0.01f);
    assert(approximately_equal(output, 100.0f));
    printf("PASS: Upper output limit\n");

    output = pid_update(&pid, -300.0f, 0.0f, 0.01f);
    assert(approximately_equal(output, -100.0f));
    printf("PASS: Lower output limit\n");

    pid_reset(&pid);
    assert(approximately_equal(pid.integral, 0.0f));
    assert(pid.first_update);
    printf("PASS: PID reset\n");

    output = pid_update(&pid, 100.0f, 70.0f, 0.0f);
    assert(approximately_equal(output, 0.0f));
    printf("PASS: Invalid time interval\n");

    printf("\nAll PID tests passed!\n");
    return 0;
}
