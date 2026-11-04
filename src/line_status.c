#include "line_status.h"

#include "pico/critical_section.h"

#define LINE_STATUS_MAX_AGE_MS 100U

static critical_section_t s_lock;

static bool s_valid = false;
static bool s_present = false;
static uint32_t s_timestamp_ms = 0U;

void line_status_init(void)
{
    critical_section_init(&s_lock);

    s_valid = false;
    s_present = false;
    s_timestamp_ms = 0U;
}

void line_status_publish(bool present, uint32_t now_ms)
{
    critical_section_enter_blocking(&s_lock);

    s_present = present;
    s_timestamp_ms = now_ms;
    s_valid = true;

    critical_section_exit(&s_lock);
}

bool line_status_is_present(uint32_t now_ms)
{
    bool valid;
    bool present;
    uint32_t timestamp_ms;

    critical_section_enter_blocking(&s_lock);

    valid = s_valid;
    present = s_present;
    timestamp_ms = s_timestamp_ms;

    critical_section_exit(&s_lock);

    if (!valid || !present)
    {
        return false;
    }

    /* Reject stale readings, including after a timer wrap. */
    return (uint32_t)(now_ms - timestamp_ms)
           <= LINE_STATUS_MAX_AGE_MS;
}