/*********************************************************************************************************************
* 文件名称          my_timestamp
* 功能说明          基于 clock_gettime 的运行耗时统计工具实现
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone          first version
* 2026-05-01        Assistant                  补充 Doxygen 注释
********************************************************************************************************************/

#include "my_timestamp.hpp"

/**
 * @brief 记录当前单调时钟作为起始时间
 */
void TimerClockGetTime::start() {
        clock_gettime(CLOCK_MONOTONIC, &start_time);
    }

/**
 * @brief 记录当前单调时钟作为结束时间
 */
void TimerClockGetTime::stop() {
    clock_gettime(CLOCK_MONOTONIC, &end_time);
}

/**
 * @brief 获取纳秒级耗时
 * @return stop 与 start 的差值，单位 ns
 */
long long TimerClockGetTime::elapsed_ns() {
    return (end_time.tv_sec - start_time.tv_sec) * 1000000000LL + 
            (end_time.tv_nsec - start_time.tv_nsec);
}

/**
 * @brief 获取微秒级耗时
 * @return stop 与 start 的差值，单位 us
 */
long long TimerClockGetTime::elapsed_us() {
    return elapsed_ns() / 1000;
}

/**
 * @brief 获取毫秒级耗时
 * @return stop 与 start 的差值，单位 ms
 */
long long TimerClockGetTime::elapsed_ms() {
    return elapsed_ns() / 1000000;
}

/**
 * @brief 获取秒级耗时
 * @return stop 与 start 的差值，单位 s
 */
double TimerClockGetTime::elapsed_sec() {
    return elapsed_ns() / 1000000000.0;
}
