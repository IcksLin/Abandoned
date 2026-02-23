/*********************************************************************************************************************
* 文件名称          navigation
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-02-22        HeavenCornerstone         
********************************************************************************************************************/

#include "navigation.hpp"

Odometer::Odometer(int32_t ppu, bool is_reversed) {
    total_distance_units = 0;
    remainder_pulses = 0;
    pulses_per_unit = ppu;
    direction_multiplier = is_reversed ? -1 : 1;
}

void Odometer::update(int16_t new_raw_pulses) {
    // 1. 根据安装方向调整脉冲符号
    int32_t current_pulses = new_raw_pulses * direction_multiplier;
    
    // 2. 将新脉冲加入余数池
    int32_t total_unprocessed = remainder_pulses + current_pulses;

    // 3. 计算进位数 (C++11标准中，整数除法向零取整，完美兼容正反转)
    int32_t carry_units = total_unprocessed / pulses_per_unit;
    
    // 4. 计算新的余数 (C++11标准中，余数符号与被除数相同)
    remainder_pulses = total_unprocessed % pulses_per_unit;

    // 5. 将进位累加到64位总里程中
    total_distance_units += carry_units;
}


PathTracker::PathTracker()
    : left_tyre(ENCODER_PPU, false)    // 初始化左轮
    , right_tyre(ENCODER_PPU, true)    // 初始化右轮（反向）
    {
    
    reset();
}

void PathTracker::reset() {
    current_index = 0;
    precise_x = 0;
    precise_y = 0;
    last_total_s = 0;
    is_recording = false;
}

void PathTracker::record_sample(int16_t left_pulses,int16_t right_pulses, float current_yaw) {
    if (!is_recording || current_index >= MAX_POINTS) return;
    left_tyre.update(left_pulses);
    right_tyre.update(right_pulses);
    
    int64_t current_total_r =  left_tyre.get_distance(); 
    int64_t current_total_l = right_tyre.get_distance();
    double current_total_s = (double)(current_total_r + current_total_l) / 2.0;
    double delta_s = current_total_s - last_total_s;
    double rad = (double)current_yaw * (M_PI / 180.0);

    precise_x += delta_s * cos(rad);
    precise_y += delta_s * sin(rad);

    path_array[current_index].x = (float)precise_x;
    path_array[current_index].y = (float)precise_y;
    path_array[current_index].yaw = current_yaw;

    last_total_s = current_total_s;
    current_index++;
}

// 生成高斯权重核
void PathTracker::generate_gaussian_kernel(float sigma, int size) {
    gaussian_kernel.clear();
    gaussian_kernel.resize(size);
    int center = size / 2;
    float sum = 0;

    for (int i = 0; i < size; i++) {
        int x = i - center;
        // 高斯一维公式: G(x) = exp(-(x^2)/(2*sigma^2))
        gaussian_kernel[i] = exp(-(float)(x * x) / (2 * sigma * sigma));
        sum += gaussian_kernel[i];
    }

    // 归一化，确保所有权重之和为 1.0，防止滤波后坐标整体缩放
    for (int i = 0; i < size; i++) {
        gaussian_kernel[i] /= sum;
    }
}

// 执行高斯卷积滤波
void PathTracker::apply_gaussian_filter(float sigma, int kernel_size) {
    if (current_index < kernel_size) return; 
    
    generate_gaussian_kernel(sigma, kernel_size);
    int radius = kernel_size / 2;

    std::vector<PathPoint> temp_path(current_index);

    for (int i = 0; i < current_index; i++) {
        float sum_x = 0, sum_y = 0;
        float sum_yaw_diff = 0; // 存储加权后的角度增量
        float center_yaw = path_array[i].yaw; // 以当前点角度为基准
        
        for (int j = 0; j < kernel_size; j++) {
            int idx = i + (j - radius);
            if (idx < 0) idx = 0;
            if (idx >= current_index) idx = current_index - 1;

            // X, Y 滤波保持不变
            sum_x += path_array[idx].x * gaussian_kernel[j];
            sum_y += path_array[idx].y * gaussian_kernel[j];
            
            // --- 处理角度环绕 ---
            float neighbor_yaw = path_array[idx].yaw;
            float diff = neighbor_yaw - center_yaw;

            // 将差值约束在 [-180, 180] 之间，寻找最短路径
            // 比如 179 到 -179 的 diff 是 -358，处理后变为 +2
            if (diff > 180.0f)  diff -= 360.0f;
            if (diff < -180.0f) diff += 360.0f;

            sum_yaw_diff += diff * gaussian_kernel[j];
        }

        temp_path[i].x = sum_x;
        temp_path[i].y = sum_y;

        // 将加权后的平滑增量加回基准角，并再次规格化到 [-180, 180]
        float filtered_yaw = center_yaw + sum_yaw_diff;
        if (filtered_yaw > 180.0f)  filtered_yaw -= 360.0f;
        if (filtered_yaw < -180.0f) filtered_yaw += 360.0f;
        
        temp_path[i].yaw = filtered_yaw;
    }

    // 回写数据
    for (int i = 0; i < current_index; i++) {
        path_array[i] = temp_path[i];
    }
}

void PathTracker::start_remember(){
    is_recording = true;
}

void PathTracker::stop_remember(bool key){
    if(key) current_index = 0;
    is_recording = false;
}