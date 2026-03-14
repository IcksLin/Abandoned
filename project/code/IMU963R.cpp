#include "IMU963R.hpp"
#include <unistd.h> // 用于 usleep

IMUHandler::IMUHandler(float a_acc, float a_gyro) 
    : alpha_acc(a_acc), alpha_gyro(a_gyro) {
    for(int i = 0; i < 3; i++) {
        acc[i] = gyro[i] = mag[i] = 0.0f;
        last_acc[i] = last_gyro[i] = 0.0f;
        gyro_offset[i] = 0.0f;
    }
}

imu_device_type_enum IMUHandler::init(void) {
    imu_device_type_enum type = imu_dev.init();
    if (type != DEV_NO_FIND) {
        // 初始化成功后自动校准
        calibrate_offsets(500); 
    }
    return type;
}

void IMUHandler::calibrate_offsets(int sample_counts) {
    float sum_g[3] = {0, 0, 0};
    
    // 累加静止时的数据
    for(int i = 0; i < sample_counts; i++) {
        sum_g[0] += (float)imu_dev.get_gyro_x() * GYRO_SCALE;
        sum_g[1] += (float)imu_dev.get_gyro_y() * GYRO_SCALE;
        sum_g[2] += (float)imu_dev.get_gyro_z() * GYRO_SCALE;
        usleep(2000); // 间隔2ms采样一次
    }

    // 计算平均零偏
    for(int i = 0; i < 3; i++) {
        gyro_offset[i] = sum_g[i] / (float)sample_counts;
    }
}

void IMUHandler::update(void) {
    // 1. 获取并转换当前瞬时值 (物理单位)
    float cur_ax = (float)imu_dev.get_acc_x() * ACC_SCALE;
    float cur_ay = (float)imu_dev.get_acc_y() * ACC_SCALE;
    float cur_az = (float)imu_dev.get_acc_z() * ACC_SCALE;

    // 2. 减去零偏 (只针对陀螺仪，加速度计通常保留重力向量)
    float cur_gx = ((float)imu_dev.get_gyro_x() * GYRO_SCALE) - gyro_offset[0];
    float cur_gy = ((float)imu_dev.get_gyro_y() * GYRO_SCALE) - gyro_offset[1];
    float cur_gz = ((float)imu_dev.get_gyro_z() * GYRO_SCALE) - gyro_offset[2];

    // 3. 一阶低通滤波 (数值平滑)
    acc[0] = alpha_acc * cur_ax + (1.0f - alpha_acc) * last_acc[0];
    acc[1] = alpha_acc * cur_ay + (1.0f - alpha_acc) * last_acc[1];
    acc[2] = alpha_acc * cur_az + (1.0f - alpha_acc) * last_acc[2];

    gyro[0] = alpha_gyro * cur_gx + (1.0f - alpha_gyro) * last_gyro[0];
    gyro[1] = alpha_gyro * cur_gy + (1.0f - alpha_gyro) * last_gyro[1];
    gyro[2] = alpha_gyro * cur_gz + (1.0f - alpha_gyro) * last_gyro[2];

    // 4. 更新滤波器状态
    for(int i = 0; i < 3; i++) {
        last_acc[i] = acc[i];
        last_gyro[i] = gyro[i];
    }

    // 5. 磁力计处理 (若有)
    if(imu_dev.imu_type == DEV_IMU963RA) {
        mag[0] = (float)imu_dev.get_mag_x();
        mag[1] = (float)imu_dev.get_mag_y();
        mag[2] = (float)imu_dev.get_mag_z();
    }
}

/**
 * @brief 航向角平滑控制器
 * @param target_yaw 目标角度 (-180 到 180)
 * @param current_yaw 当前解算出的角度 (-180 到 180)
 * @param max_output 最大输出限制 (例如 25.0)
 * @return float 映射到 [-max_output, max_output] 的电机控制增量
 */
float calculate_yaw_control(float target_yaw, float current_yaw, float max_output) {
    // 1. 计算最短路径误差 (解决 -180/180 临界点问题)
    // 这里的 error 范围会被锁定在 [-180, 180]
    float error = target_yaw - current_yaw;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    // 2. 归一化误差到 [-1, 1] 范围，用于 Sigmoid 计算
    const float range = 45.0f; 
    float normalized_error = error / range; 

    // 3. 使用 Sigmoid 函数进行平滑映射
    // k 控制响应灵敏度：k 越大，中位附近的修正越猛；k 越小，过渡越平缓
    float k = 4.0f; 
    float sigmoid = (2.0f / (1.0f + exp(-k * normalized_error))) - 1.0f;

    // 4. 将映射结果 [ -1, 1 ] 放大到电机控制量 [ -25, 25 ]
    float output = sigmoid * max_output;

    // 5. 边界保护
    output = std::max(-max_output, std::min(max_output, output));

    return output;
}