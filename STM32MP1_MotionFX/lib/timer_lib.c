#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "timer_lib.h"

#define MAX_TIMER_COUNT 1000

struct timer_node {
    int fd;
    time_handler callback;
    void *user_data;
    unsigned int interval;
    t_timer type;
    struct timer_node *next;
};

static void *_timer_thread(void *data);
static pthread_t g_thread_id;
static struct timer_node *g_head;

int timer_initialize(void)
{
    return pthread_create(&g_thread_id, NULL, _timer_thread, NULL);
}

size_t start_timer(unsigned int interval, int flags,
                   time_handler handler, t_timer type, void *user_data)
{
    struct timer_node *new_node = NULL;
    struct itimerspec new_value;

    new_node = (struct timer_node *)malloc(sizeof(struct timer_node));
    if(!new_node)
        return -ENOMEM;

    new_node->callback = handler;
    new_node->user_data = user_data;
    new_node->interval = interval;
    new_node->type = type;

    new_node->fd = timerfd_create(flags, 0);
    if (new_node->fd == -1)
        goto err_free;

    new_value.it_value.tv_sec = interval / 1000;
    new_value.it_value.tv_nsec = (interval % 1000)* 1000000;

    if (type == TIMER_PERIODIC) {
      new_value.it_interval.tv_sec= interval / 1000;
      new_value.it_interval.tv_nsec = (interval %1000) * 1000000;
    } else {
      new_value.it_interval.tv_sec= 0;
      new_value.it_interval.tv_nsec = 0;
    }

    timerfd_settime(new_node->fd, 0, &new_value, NULL);

    /*Inserting the timer node into the list*/
    new_node->next = g_head;
    g_head = new_node;

    return (size_t)new_node;

err_free:
    free(new_node);

    return 0;
}

void stop_timer(size_t timer_id)
{
    struct timer_node *tmp = NULL;
    struct timer_node *node = (struct timer_node *)timer_id;

    if (node == NULL)
        return;

    if(node == g_head) {
        g_head = g_head->next;
    } else {
        tmp = g_head;

        while(tmp && tmp->next != node)
            tmp = tmp->next;

        if(tmp) {
            tmp->next = tmp->next->next;
            close(node->fd);
            free(node);
        }
    }
}

void timer_finalize(void)
{
    while(g_head)
        stop_timer((size_t)g_head);

    pthread_cancel(g_thread_id);
    pthread_join(g_thread_id, NULL);
}

struct timer_node *_get_timer_from_fd(int fd)
{
    struct timer_node *tmp = g_head;

    while(tmp) {
        if(tmp->fd == fd)
            return tmp;

        tmp = tmp->next;
    }

    return NULL;
}

void *_timer_thread(void * data)
{
    struct pollfd ufds[MAX_TIMER_COUNT] = {{0}};
    struct timer_node *tmp = NULL;
    int read_fds = 0, i, s;
    int iMaxCount = 0;
    uint64_t exp;

    while(1) {
        pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
        pthread_testcancel();
        pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL);

        iMaxCount = 0;
        tmp = g_head;

        memset(ufds, 0, sizeof(struct pollfd)*MAX_TIMER_COUNT);
        while(tmp) {
            ufds[iMaxCount].fd = tmp->fd;
            ufds[iMaxCount].events = POLLIN;
            iMaxCount++;

            tmp = tmp->next;
        }

        read_fds = poll(ufds, iMaxCount, 100);
        if (read_fds <= 0)
            continue;

        for (i = 0; i < iMaxCount; i++) {
            if (ufds[i].revents & POLLIN) {
                s = read(ufds[i].fd, &exp, sizeof(uint64_t));
                if (s != sizeof(uint64_t))
                    continue;

                tmp = _get_timer_from_fd(ufds[i].fd);
                if(tmp && tmp->callback)
                    tmp->callback((size_t)tmp, tmp->user_data);
            }
        }
    }

    return NULL;
}
