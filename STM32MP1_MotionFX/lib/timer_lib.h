#ifndef __TIME_H
#define __TIME_H

#include <stdlib.h>
#include <sys/timerfd.h>

typedef enum {
    TIMER_SINGLE_SHOT = 0,
    TIMER_PERIODIC,
} t_timer;

typedef void (*time_handler)(size_t timer_id, void * user_data);

int timer_initialize(void);
size_t start_timer(unsigned int interval, int flags,
                   time_handler handler, t_timer type,
                   void *user_data);
void stop_timer(size_t timer_id);
void timer_finalize(void);

#endif /* __TIME_H */
