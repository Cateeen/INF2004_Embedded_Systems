#ifndef MOCK_CRITICAL_SECTION_H
#define MOCK_CRITICAL_SECTION_H

#include <pthread.h>

typedef struct {
    pthread_mutex_t mutex;
} critical_section_t;

static inline void critical_section_init(critical_section_t *cs)
{
    pthread_mutex_init(&cs->mutex, NULL);
}

static inline void critical_section_enter_blocking(critical_section_t *cs)
{
    pthread_mutex_lock(&cs->mutex);
}

static inline void critical_section_exit(critical_section_t *cs)
{
    pthread_mutex_unlock(&cs->mutex);
}

#endif