/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/


#ifndef __MY_TIMESTAMP_HPP__
#define __MY_TIMESTAMP_HPP__
//时间戳文件，用于统计程序运行时间
#include <time.h>
#include <iostream>
#include <chrono>

// 方法1：使用clock_gettime（纳秒级精度）
class TimerClockGetTime {
private:
    struct timespec start_time, end_time;
    
public:
    void start();
    
    void stop();
    
    // 获取经过的时间（纳秒）
    long long elapsed_ns();
    // 获取经过的时间（微秒）
    long long elapsed_us();
    
    // 获取经过的时间（毫秒）
    long long elapsed_ms();
    
    // 获取经过的时间（秒）
    double elapsed_sec();
};


#endif