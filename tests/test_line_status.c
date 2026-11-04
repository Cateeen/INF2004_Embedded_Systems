#include "line_status.h"

#include <assert.h>
#include <stdio.h>
#include <stdint.h>

int main(void)
{
    line_status_init();

    /* No initial reading: fail safe. */
    assert(!line_status_is_present(0U));

    /* A newly detected line is present. */
    line_status_publish(true, 1000U);
    assert(line_status_is_present(1000U));
    assert(line_status_is_present(1050U));
    assert(line_status_is_present(1100U));

    /* Reject a reading older than 100 ms. */
    assert(!line_status_is_present(1101U));

    /* A fresh negative observation means no line. */
    line_status_publish(false, 1200U);
    assert(!line_status_is_present(1200U));

    /* A fresh positive observation restores line presence. */
    line_status_publish(true, 1300U);
    assert(line_status_is_present(1350U));

    /* Check unsigned timestamp wraparound. */
    line_status_publish(true, UINT32_MAX - 49U);
    assert(line_status_is_present(25U));
    assert(!line_status_is_present(100U));

    printf("PASS: line status freshness and expiry\n");

    return 0;
}