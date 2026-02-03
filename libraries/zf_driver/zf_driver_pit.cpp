#include "zf_driver_pit.hpp"

#include <unistd.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <sched.h>
#include <sys/prctl.h>
#include <pthread.h>
#include <stdlib.h>

zf_driver_pit_rt::zf_driver_pit_rt()
    : exit_flag(false), running(false),
      timer_fd(-1), event_fd(-1), epoll_fd(-1),
      thread(0), period_ms(0),
      user_cb_void(nullptr), user_cb_count(nullptr),
      thread_priority(0),
      use_realtime(false), bound_core(-1),
      clock_raw(false), mem_locked(false)
{
}

zf_driver_pit_rt::~zf_driver_pit_rt()
{
    stop();
}

static int create_timerfd_helper(clockid_t clk, uint32_t period_ms_arg)
{
    int fd = timerfd_create(clk, TFD_NONBLOCK | TFD_CLOEXEC);
    if (fd < 0) return -1;

    struct itimerspec its;
    memset(&its, 0, sizeof(its));
    its.it_value.tv_sec = period_ms_arg / 1000;
    its.it_value.tv_nsec = (period_ms_arg % 1000) * 1000000;
    if (its.it_value.tv_sec == 0 && its.it_value.tv_nsec == 0) {
        its.it_value.tv_nsec = 1;
    }
    its.it_interval.tv_sec = period_ms_arg / 1000;
    its.it_interval.tv_nsec = (period_ms_arg % 1000) * 1000000;

    if (timerfd_settime(fd, 0, &its, NULL) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static int create_eventfd_helper()
{
    int fd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    return fd;
}

static int create_epoll_helper()
{
    return epoll_create1(EPOLL_CLOEXEC);
}

int zf_driver_pit_rt::init_ms(uint32_t period_ms_arg,
                              void (*callback)(void),
                              int priority,
                              bool realtime,
                              int cpu_core,
                              bool use_raw_clock,
                              bool lock_memory)
{
    // forward to the common implementation but set the appropriate callback slot
    // We implement core logic in this file; to avoid duplication call the other init.
    return init_ms_with_count(period_ms_arg,
                              nullptr, // no count callback
                              priority, realtime, cpu_core, use_raw_clock, lock_memory) == 0
           ? ( (user_cb_void = callback), 0 ) : -1;
}

int zf_driver_pit_rt::init_ms_with_count(uint32_t period_ms_arg,
                                         zf_pit_rt_callback_fun callback,
                                         int priority,
                                         bool realtime,
                                         int cpu_core,
                                         bool use_raw_clock,
                                         bool lock_memory)
{
    if (callback == nullptr && user_cb_void == nullptr && callback == nullptr) {
        // allow user to later set callback via init_ms; if both null, error
        // but for safe behavior: if both null and we were called directly, we proceed only if legacy callback will be set.
    }

    if (period_ms_arg < ZF_PIT_MIN_PERIOD_MS || period_ms_arg > ZF_PIT_MAX_PERIOD_MS) {
        fprintf(stderr, "pit_rt: period %u out of range [%d, %d]\n",
                period_ms_arg, ZF_PIT_MIN_PERIOD_MS, ZF_PIT_MAX_PERIOD_MS);
        return -1;
    }
    if (running.load()) {
        fprintf(stderr, "pit_rt: already running\n");
        return -1;
    }

    // store config
    period_ms = period_ms_arg;
    user_cb_count = callback;
    user_cb_void = nullptr; // if you want void callback, call init_ms(...) instead
    thread_priority = priority;
    use_realtime = realtime;
    bound_core = cpu_core;
    clock_raw = use_raw_clock;

    if (lock_memory) {
        if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
            perror("mlockall failed (continuing without locked memory)");
            mem_locked = false;
        } else {
            mem_locked = true;
        }
    }

    // Predeclare variables to avoid goto jumping over initializations
    int ret = 0;

    clockid_t clk = clock_raw ? CLOCK_MONOTONIC_RAW : CLOCK_MONOTONIC;
    timer_fd = create_timerfd_helper(clk, period_ms);
    if (timer_fd < 0) {
        perror("timerfd_create/settime failed");
        ret = -1;
        goto fail;
    }

    event_fd = create_eventfd_helper();
    if (event_fd < 0) {
        perror("eventfd failed");
        ret = -1;
        goto fail;
    }

    epoll_fd = create_epoll_helper();
    if (epoll_fd < 0) {
        perror("epoll_create1 failed");
        ret = -1;
        goto fail;
    }

    struct epoll_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN;
    ev.data.fd = timer_fd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_fd, &ev) < 0) {
        perror("epoll_ctl add timer_fd failed");
        ret = -1;
        goto fail;
    }
    memset(&ev, 0, sizeof(ev));
    ev.events = EPOLLIN;
    ev.data.fd = event_fd;
    if (epoll_ctl(epoll_fd, EPOLL_CTL_ADD, event_fd, &ev) < 0) {
        perror("epoll_ctl add event_fd failed");
        ret = -1;
        goto fail;
    }

    // Try to create thread with requested realtime attributes if requested
    if (use_realtime && thread_priority > 0) {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
        struct sched_param sp;
        memset(&sp, 0, sizeof(sp));
        int minp = sched_get_priority_min(SCHED_FIFO);
        int maxp = sched_get_priority_max(SCHED_FIFO);
        if (minp == -1 || maxp == -1) { minp = 1; maxp = 99; }
        if (thread_priority < minp) sp.sched_priority = minp;
        else if (thread_priority > maxp) sp.sched_priority = maxp;
        else sp.sched_priority = thread_priority;
        pthread_attr_setschedparam(&attr, &sp);

        ret = pthread_create(&thread, &attr, thread_entry, this);
        pthread_attr_destroy(&attr);
        if (ret != 0) {
            if (ret != EPERM) {
                fprintf(stderr, "pthread_create (with attr) failed: %s\n", strerror(ret));
                ret = -1;
                goto fail;
            }
            // EPERM: fall through to create without attributes and attempt post-create setschedparam
        } else {
            // set affinity if requested
            if (bound_core >= 0) {
                cpu_set_t cpus;
                CPU_ZERO(&cpus);
                CPU_SET(bound_core, &cpus);
                if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpus) != 0) {
                    perror("pthread_setaffinity_np (post-create) failed");
                }
            }
            running.store(true);
            return 0;
        }
    }

    // create thread without special attr
    ret = pthread_create(&thread, NULL, thread_entry, this);
    if (ret != 0) {
        fprintf(stderr, "pthread_create failed: %s\n", strerror(ret));
        ret = -1;
        goto fail;
    }

    // Try to set realtime scheduling post-create if requested
    if (use_realtime && thread_priority > 0) {
        struct sched_param sp2;
        memset(&sp2, 0, sizeof(sp2));
        int minp = sched_get_priority_min(SCHED_FIFO);
        int maxp = sched_get_priority_max(SCHED_FIFO);
        if (minp == -1 || maxp == -1) { minp = 1; maxp = 99; }
        if (thread_priority < minp) sp2.sched_priority = minp;
        else if (thread_priority > maxp) sp2.sched_priority = maxp;
        else sp2.sched_priority = thread_priority;

        if (pthread_setschedparam(thread, SCHED_FIFO, &sp2) != 0) {
            perror("pthread_setschedparam (post-create) failed (need root/CAP_SYS_NICE?)");
            // continue — thread exists, just not realtime
        }
    }

    if (bound_core >= 0) {
        cpu_set_t cpus;
        CPU_ZERO(&cpus);
        CPU_SET(bound_core, &cpus);
        if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpus) != 0) {
            perror("pthread_setaffinity_np failed");
        }
    }

    running.store(true);
    return 0;

fail:
    if (timer_fd >= 0) { close(timer_fd); timer_fd = -1; }
    if (event_fd >= 0) { close(event_fd); event_fd = -1; }
    if (epoll_fd >= 0) { close(epoll_fd); epoll_fd = -1; }
    if (mem_locked) {
        munlockall();
        mem_locked = false;
    }
    return -1;
}

void zf_driver_pit_rt::stop()
{
    if (!running.load()) return;

    exit_flag.store(true);

    if (event_fd >= 0) {
        uint64_t one = 1;
        ssize_t s = write(event_fd, &one, sizeof(one));
        (void)s;
    }

    if (thread != 0) {
        pthread_join(thread, NULL);
        thread = 0;
    }

    if (timer_fd >= 0) { close(timer_fd); timer_fd = -1; }
    if (event_fd >= 0) { close(event_fd); event_fd = -1; }
    if (epoll_fd >= 0) { close(epoll_fd); epoll_fd = -1; }

    if (mem_locked) {
        munlockall();
        mem_locked = false;
    }

    user_cb_void = nullptr;
    user_cb_count = nullptr;
    running.store(false);
    exit_flag.store(false);
}

int zf_driver_pit_rt::set_priority(int priority, bool realtime)
{
    if (thread == 0) {
        thread_priority = priority;
        use_realtime = realtime;
        return 0;
    }

    if (!realtime || priority <= 0) {
        struct sched_param sp;
        memset(&sp, 0, sizeof(sp));
        if (pthread_setschedparam(thread, SCHED_OTHER, &sp) != 0) {
            perror("pthread_setschedparam -> SCHED_OTHER failed");
            return -1;
        }
        thread_priority = priority;
        use_realtime = false;
        return 0;
    }

    struct sched_param sp2;
    memset(&sp2, 0, sizeof(sp2));
    int minp = sched_get_priority_min(SCHED_FIFO);
    int maxp = sched_get_priority_max(SCHED_FIFO);
    if (minp == -1 || maxp == -1) { minp = 1; maxp = 99; }
    if (priority < minp) sp2.sched_priority = minp;
    else if (priority > maxp) sp2.sched_priority = maxp;
    else sp2.sched_priority = priority;

    if (pthread_setschedparam(thread, SCHED_FIFO, &sp2) != 0) {
        perror("pthread_setschedparam failed");
        return -1;
    }
    thread_priority = priority;
    use_realtime = true;
    return 0;
}

int zf_driver_pit_rt::set_affinity(int core_id)
{
    if (core_id < 0) return -1;
    if (thread == 0) {
        bound_core = core_id;
        return 0;
    }
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(core_id, &cpus);
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpus) != 0) {
        perror("pthread_setaffinity_np failed");
        return -1;
    }
    bound_core = core_id;
    return 0;
}

void *zf_driver_pit_rt::thread_entry(void *arg)
{
    zf_driver_pit_rt *self = static_cast<zf_driver_pit_rt*>(arg);

    prctl(PR_SET_NAME, "zf_pit_rt", 0, 0, 0);

    const int MAX_EVENTS = 2;
    struct epoll_event events[MAX_EVENTS];

    while (!self->exit_flag.load()) {
        int n = epoll_wait(self->epoll_fd, events, MAX_EVENTS, -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            perror("epoll_wait failed");
            break;
        }
        for (int i = 0; i < n; ++i) {
            int fd = events[i].data.fd;
            if (fd == self->event_fd) {
                uint64_t v;
                while (read(self->event_fd, &v, sizeof(v)) > 0) { }
                if (self->exit_flag.load()) break;
            } else if (fd == self->timer_fd) {
                // consume all expirations
                uint64_t expirations = 0;
                ssize_t r;
                do {
                    r = read(self->timer_fd, &expirations, sizeof(expirations));
                    if (r > 0) {
                        // Prefer the new callback with expirations if set, otherwise call the legacy void callback
                        if (self->user_cb_count) {
                            self->user_cb_count(expirations);
                        } else if (self->user_cb_void) {
                            self->user_cb_void();
                        }
                    }
                } while (r > 0);
                if (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    perror("read(timer_fd) failed");
                }
            }
        }
    }

    pthread_exit(NULL);
    return nullptr;
}