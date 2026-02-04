#ifndef __IMU_HANDLER_HPP__
#define __IMU_HANDLER_HPP__

#include "zf_device_imu.hpp"
#include <vector>

/**
 * @brief IMU数据处理类
 * 集成：Linux IIO原生数据读取、一阶低通滤波(LPF)、静态零偏校准
 */
class IMUHandler {
public:
    // 处理后的物理量数据（对外只读，保证安全）
    float acc[3];   // 加速度 (m/s^2)
    float gyro[3];  // 角速度 (deg/s)
    float mag[3];   // 磁力计原始值

    /**
     * @brief 构造函数，初始化滤波器参数
     * @param a_acc  加速度计滤波系数 (0.0~1.0)，越小滤波越强，延迟越大
     * @param a_gyro 陀螺仪滤波系数 (0.0~1.0)，越小滤波越强，延迟越大
     */
    IMUHandler(float a_acc = 0.1f, float a_gyro = 0.5f);

    /**
     * @brief 初始化硬件并自动进行零偏校准
     * @return imu_device_type_enum 识别到的设备型号
     */
    imu_device_type_enum init(void);

    /**
     * @brief 核心更新函数，建议在 1ms 或 2ms 定时任务中调用
     * 流程：读取原始值 -> 归一化 -> 减去零偏 -> 低通滤波
     */
    void update(void);

    /**
     * @brief 手动触发零偏校准（必须在小车完全静止时调用）
     * @param sample_counts 校准采样次数，默认500次
     */
    void calibrate_offsets(int sample_counts = 500);

private:
    zf_device_imu imu_dev;          // 逐飞科技底层的IMU设备对象
    
    float alpha_acc;                // 加速度计滤波系数
    float alpha_gyro;               // 陀螺仪滤波系数
    
    float gyro_offset[3];           // 存储静止时的陀螺仪零偏
    float last_acc[3];              // 滤波器状态：上一时刻加速度
    float last_gyro[3];             // 滤波器状态：上一时刻角速度

    // 内部单位换算常量
    const float ACC_SCALE = 16.0f * 9.8f / 32768.0f;
    const float GYRO_SCALE = 2000.0f / 32768.0f;
};

#endif