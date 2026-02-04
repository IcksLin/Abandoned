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