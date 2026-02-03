/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/



#include "my_timestamp.hpp"

void TimerClockGetTime::start() {
        clock_gettime(CLOCK_MONOTONIC, &start_time);
    }

void TimerClockGetTime::stop() {
    clock_gettime(CLOCK_MONOTONIC, &end_time);
}

// 获取经过的时间（纳秒）
long long TimerClockGetTime::elapsed_ns() {
    return (end_time.tv_sec - start_time.tv_sec) * 1000000000LL + 
            (end_time.tv_nsec - start_time.tv_nsec);
}

// 获取经过的时间（微秒）
long long TimerClockGetTime::elapsed_us() {
    return elapsed_ns() / 1000;
}

// 获取经过的时间（毫秒）
long long TimerClockGetTime::elapsed_ms() {
    return elapsed_ns() / 1000000;
}

// 获取经过的时间（秒）
double TimerClockGetTime::elapsed_sec() {
    return elapsed_ns() / 1000000000.0;
}