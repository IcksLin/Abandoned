#ifndef ZF_DRIVER_PIT_RT_HPP
#define ZF_DRIVER_PIT_RT_HPP

#include <stdint.h>
#include <pthread.h>
#include <atomic>

#define ZF_PIT_MIN_PERIOD_MS 1
#define ZF_PIT_MAX_PERIOD_MS 60000

// 兼容原有项目中可能存在的 typedef:
// 原项目的回调签名通常为: typedef void (*pit_callback_fun)(void);
// 我们不重新定义同名以避免冲突，而是使用两种回调类型：
// 1) 兼容型：使用项目中可能已定义的 pit_callback_fun（void(void))
//    -- 该类型在本头不重复定义，编译器会使用任一已见过的定义。
// 2) 新型：带过期计数参数，便于用户获知 missed/tick count。
typedef void (*zf_pit_rt_callback_fun)(uint64_t expirations);

class zf_driver_pit_rt
{
public:
    zf_driver_pit_rt();
    ~zf_driver_pit_rt();

    // 兼容现有签名（void callback(void)）
    // priority: realtime priority (1..99). If <=0, no realtime requested.
    // cpu_core: if >=0 bind to that core
    // use_raw_clock: use CLOCK_MONOTONIC_RAW when true
    // lock_memory: mlockall if true
    int init_ms(uint32_t period_ms,
                void (*callback)(void),
                int priority = 0,
                bool realtime = false,
                int cpu_core = -1,
                bool use_raw_clock = false,
                bool lock_memory = true);

    // 新接口：回调接收 expirations（uint64_t）
    int init_ms_with_count(uint32_t period_ms,
                           zf_pit_rt_callback_fun callback,
                           int priority = 0,
                           bool realtime = false,
                           int cpu_core = -1,
                           bool use_raw_clock = false,
                           bool lock_memory = true);

    void stop();

    int set_priority(int priority, bool realtime);
    int set_affinity(int core_id);

    bool is_running() const { return running.load(); }

private:
    // non-copyable
    zf_driver_pit_rt(const zf_driver_pit_rt&) = delete;
    zf_driver_pit_rt& operator=(const zf_driver_pit_rt&) = delete;

    static void *thread_entry(void *arg);

private:
    std::atomic<bool> exit_flag;
    std::atomic<bool> running;
    int timer_fd;
    int event_fd;
    int epoll_fd;
    pthread_t thread;
    uint32_t period_ms;
    // two callback storage: one for legacy void(void), one for new uint64_t param
    void (*user_cb_void)(void);
    zf_pit_rt_callback_fun user_cb_count;
    int thread_priority;
    bool use_realtime;
    int bound_core;
    bool clock_raw;
    bool mem_locked;
};

#endif // ZF_DRIVER_PIT_RT_HPP