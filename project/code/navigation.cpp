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
    , right_tyre(ENCODER_PPU, false)    // 初始化右轮（反向）
    {
    
    reset();
}

void PathTracker::reset() {
    current_index = 0;
    //重置内部计算用累计变量
    precise_x = 0;
    precise_y = 0;
    last_total_s = 0;
    //重置开关
    is_recording = false;
    is_reproduction = false;
    //重置目标点与当前位置
    current_location = {0, 0.0f, 0.0f};
    target_point = {0, 0.0f, 0.0f};
    //重置里程计
    left_tyre.reset();
    right_tyre.reset();
}

// 此函数与陀螺仪数据更新同步执行
void PathTracker::record_sample( float current_yaw) {
    if (!is_recording || current_index >= MAX_POINTS) return;

    
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

void PathTracker::get_location(float current_yaw) {
    if(is_recording) return;

    int64_t current_total_r =  left_tyre.get_distance(); 
    int64_t current_total_l = right_tyre.get_distance();
    double current_total_s = (double)(current_total_r + current_total_l) / 2.0;
    double delta_s = current_total_s - last_total_s;

    if (!std::isfinite(current_yaw) || !std::isfinite(delta_s)) {
        return;
    }

    double rad = (double)current_yaw * (M_PI / 180.0);
    
    double dx = delta_s * cos(rad);
    double dy = delta_s * sin(rad);

    precise_x += dx;
    precise_y += dy;

    current_location.x = static_cast<float>(precise_x);
    current_location.y = static_cast<float>(precise_y);
    last_total_s = current_total_s;
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
    if(key) reset();
    is_recording = false;
}

/**
 * @brief 从二进制文件一键载入内存
 */
bool PathTracker::load_binary_map(const std::string& bin_filename) {
    std::ifstream ifs(bin_filename, std::ios::binary | std::ios::ate);
    if (!ifs.is_open()) return false;

    std::streamsize size = ifs.tellg();
    ifs.seekg(0, std::ios::beg);

    int count = size / sizeof(MapPoint);
    tracking_map.resize(count);

    if (ifs.read(reinterpret_cast<char*>(tracking_map.data()), size)) {
        last_closest_idx = 0; // 加载成功，重置索引起点
        return true;
    }
    return false;
}

/**
 * @brief 单向增量搜索算法
 * 满足：1. 不找已行驶过的点 2. 倒车时索引锁死在原地 3. 高效O(k)计算
 */
int PathTracker::find_closest_index() {
    if (tracking_map.empty()) return 0;

    int best_idx = last_closest_idx;
    
    auto get_dist_sq = [&](const MapPoint& p) {
        return (p.x - current_location.x) * (p.x - current_location.x) + (p.y - current_location.y) * (p.y - current_location.y);
    };

    float min_dist_sq = get_dist_sq(tracking_map[last_closest_idx]);

    // 搜索窗口：从上一次的位置开始，向前搜索固定范围
    int search_end = std::min((int)tracking_map.size() - 1, last_closest_idx + FORWARD_WINDOW);

    // 【单向性实现】：i 从 last_closest_idx + 1 开始，不走回头路
    for (int i = last_closest_idx + 1; i <= search_end; ++i) {
        float d_sq = get_dist_sq(tracking_map[i]);
        if (d_sq < min_dist_sq) {
            min_dist_sq = d_sq;
            best_idx = i;
        }
    }

    // 更新全局最近索引。如果倒车，min_dist_sq 始终是 last_closest_idx 最近，
    // 则 best_idx 不会更新，满足“拉回小车”的需求。
    last_closest_idx = best_idx;
    return last_closest_idx;
}

/**
 * @brief 获取前方目标点
 */
bool PathTracker::get_look_ahead_point(int look_ahead_dist_idx) {
    // 1. 检查地图是否有效
    if (tracking_map.empty()) return false;

    // 2. 计算目标索引，确保不越界
    int target_idx = last_closest_idx + look_ahead_dist_idx;
    if (target_idx >= (int)tracking_map.size()) {
        target_idx = (int)tracking_map.size() - 1;
    }

    // 3. 核心：将结果更新到类成员 target_point，打通数据流
    target_point = tracking_map[target_idx];

    return true;
}

float PathTracker::calculate_target_yaw() {
    // 1. 计算坐标差值
    float dx = target_point.x - current_location.x;
    float dy = target_point.y - current_location.y;

    // 2. 使用 atan2 获取弧度制方向角
    // atan2 返回值范围是 (-pi, pi]
    float angle_rad = std::atan2(dy, dx);

    // 3. 弧度转角度
    float angle_deg = angle_rad * (180.0f / (float)M_PI);

    // 4. (可选) 如果你的陀螺仪坐标系 0 度指向北方且顺时针为正，
    // 需要在这里进行坐标系转换。
    // 如果是标准笛卡尔坐标系（0度指东，逆时针为正），则直接返回 angle_deg。
    
    return angle_deg; 
}