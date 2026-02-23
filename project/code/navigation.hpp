/*********************************************************************************************************************
* 文件名称          navigation
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-02-22        HeavenCornerstone         
********************************************************************************************************************/
#ifndef __NAVIGATION_HPP__
#define __NAVIGATION_HPP__

#include <stdint.h>
#include <math.h>
#include <vector>
#include "akima.hpp"

#define ENCODER_PPU 100

class Odometer {
private:
    int64_t total_distance_units; // 累计的完整里程（如：毫米或厘米）
    int32_t remainder_pulses;     // 尚未进位的脉冲余数池
    int32_t pulses_per_unit;      // 产生1个单位里程所需要的脉冲数（即你的“地板除除数”）
    int8_t  direction_multiplier; // 方向乘子 

public:
    // 构造函数
    // ppu: pulses_per_unit 地板除参数
    // is_reversed: 是否需要反向记录
    Odometer(int32_t ppu, bool is_reversed = false);

    // 核心功能：更新里程计（每次定时器中断调用）
    // ⚠️ 注意：传入的必须是原始脉冲数 new_raw_pulses，而不是浮点数速度
    void update(int16_t new_raw_pulses);

    // 获取当前累计的绝对精确里程（整数部分）
    int64_t get_distance() const {
        return total_distance_units;
    }
    
    // （可选）如果导航算法需要当前最高精度的瞬时里程（包含余数转换的小数）
    double get_exact_distance_float() const {
        return (double)total_distance_units + ((double)remainder_pulses / pulses_per_unit);
    }
    
    // 清零里程计（用于重新标定起点）
    void reset() {
        total_distance_units = 0;
        remainder_pulses = 0;
    }
};

// 路径点结构体 (占用 12 字节)
#pragma pack(push, 1)
struct PathPoint {
    float x;      // 当前笛卡尔坐标 X (基于里程计计算出的脉冲单位)
    float y;      // 当前笛卡尔坐标 Y (基于里程计计算出的脉冲单位)
    float yaw;    // 记录时的绝对角度 (度)
};
#pragma pack(pop)

class PathTracker {
public:
    static const int MAX_POINTS = 4 * 60 * 100; // 10ms采样，4分钟共24000点
    PathPoint path_array[MAX_POINTS];
    int current_index = 0;
    bool is_recording = false;

    // 坐标累积变量
    double precise_x = 0;
    double precise_y = 0;
    double last_total_s = 0; 

    //左右轮里程计
    Odometer left_tyre;
    Odometer right_tyre;

    PathTracker();

    void reset();
    void record_sample(int16_t left_pulses,int16_t right_pulses, float current_yaw);

    /**
     * 对当前已记录的 path_array 进行高斯平滑
     * @param sigma 高斯标准差，越大越平滑（建议 1.0 - 2.0）
     * @param kernel_size 核大小，必须为奇数（建议 5 或 7）
     */
    void apply_gaussian_filter(float sigma, int kernel_size);

    /**
     * 开始录制路径，从当前已有的current_index上追加录制
     */
    void start_remember();

    /**
     * 停止录制，可清楚录制点
     * @param key 是否清楚current_index
     */
    void stop_remember(bool key);

private:
    std::vector<float> gaussian_kernel;
    void generate_gaussian_kernel(float sigma, int size);
};

#endif