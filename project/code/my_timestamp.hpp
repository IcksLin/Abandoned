/*********************************************************************************************************************
* 文件名称          my_timestamp
* 功能说明          基于 clock_gettime 的运行耗时统计工具
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone          first version
* 2026-05-01        Assistant                  补充 Doxygen 注释
********************************************************************************************************************/


#ifndef __MY_TIMESTAMP_HPP__
#define __MY_TIMESTAMP_HPP__
#include <time.h>
#include <iostream>
#include <chrono>

/**
 * @brief 单段耗时统计器
 * @details 使用 CLOCK_MONOTONIC 记录起止时间，适合测试线程周期、图像处理耗时和控制器耗时。
 * @note 该类不做线程同步，多个线程共用同一个对象时需要外部保证访问顺序。
 */
class TimerClockGetTime {
private:
    struct timespec start_time, end_time; ///< 起止时间戳，单位由系统 timespec 表示
    
public:
    /**
     * @brief 记录起始时间
     */
    void start();
    
    /**
     * @brief 记录结束时间
     */
    void stop();
    
    /**
     * @brief 获取 start 到 stop 之间的耗时
     * @return 纳秒级耗时
     */
    long long elapsed_ns();

    /**
     * @brief 获取 start 到 stop 之间的耗时
     * @return 微秒级耗时
     */
    long long elapsed_us();
    
    /**
     * @brief 获取 start 到 stop 之间的耗时
     * @return 毫秒级耗时
     */
    long long elapsed_ms();
    
    /**
     * @brief 获取 start 到 stop 之间的耗时
     * @return 秒级耗时，带小数
     */
    double elapsed_sec();
};


#endif
