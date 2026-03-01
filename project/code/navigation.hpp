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
#include <string>
#include <fstream>
#include "akima.hpp"

#define ENCODER_PPU 10

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
    // 路径录制相关变量
    static const int MAX_POINTS = 4 * 60 * 100; // 10ms采样，4分钟共24000点
    PathPoint path_array[MAX_POINTS];
    int current_index = 0;
    bool is_recording = false;

    // 路径复现相关变量 
    std::vector<MapPoint> tracking_map; // 存放从 .bin 加载的平滑后地图
    int last_closest_idx = 0;           // 核心：上一帧匹配到的索引
    const int FORWARD_WINDOW = 200;     // 单向搜索窗口大小（可根据速度动态调整）
    float current_location[2];          // 记录当前位置
    bool is_reproduction = false;       

    // 坐标累积变量
    double precise_x = 0;
    double precise_y = 0;
    double last_total_s = 0; 

    //左右轮里程计，用于路径录制
    Odometer left_tyre;
    Odometer right_tyre;

    PathTracker();

    /**
     * @brief 状态重置函数
     * 重置状态量包括：坐标累计量，当前索引值，x位置累计值，y位置累计值
     *               路径记录开关，路径复现开关，未在该函数中处理的变量不会重置
     *               使用时需要注意
     */
    void reset();

    /**
     * @brief 路径记录函数
     * @param current_yaw 当前角度
     */
    void record_sample(float current_yaw);

    /**
     * @brief 位置计算函数
     * @param current_yaw 当前角度
     */
    void get_location(float current_yaw);

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

    /**
     * @brief 从二进制文件加载地图
     * @param bin_filename .bin文件路径
     * @return bool 是否加载成功
     */
    bool load_binary_map(const std::string& bin_filename);

    /**
     * @brief 单向增量搜索最近点
     * @param cur_x 当前小车惯导坐标X
     * @param cur_y 当前小车惯导坐标Y
     * @return int 匹配到的地图索引
     */
    int find_closest_index(float cur_x, float cur_y);

    /**
     * @brief 获取预瞄点 (用于控制算法)
     * @param look_ahead_dist_idx 预瞄点相对于最近点的索引偏移
     * @return MapPoint 目标点坐标
     */
    MapPoint get_look_ahead_point(int look_ahead_dist_idx);

    /**
     * @brief 计算从当前点指向目标点的期望航向角
     * @param cur_x 当前小车 X 坐标
     * @param cur_y 当前小车 Y 坐标
     * @param target_x 预瞄目标点 X 坐标
     * @param target_y 预瞄目标点 Y 坐标
     * @return float 期望角度 (-180 到 180)
     */
    float calculate_target_yaw(float cur_x, float cur_y, float target_x, float target_y);

private:
    std::vector<float> gaussian_kernel;
    void generate_gaussian_kernel(float sigma, int size);
};

#endif