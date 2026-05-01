/**
 * @file config_setting.cpp
 * @brief 控制器参数文件读写、上位机调参解析与控制器同步实现
 */
#include "config_seting.hpp"

#include <fstream>
#include <string>
#include <cstring>
#include <cstdlib>

uint8_t control_model = 1; // 控制模式，0 PID，1 LADRC

// 参数缓存区：所有上位机和配置文件参数先进入这些变量，再统一写入控制器对象。
float lardc_r_h = 0.001f, lardc_r_r = 150.0f, lardc_r_wc = 1.0f, lardc_r_w0 = 120.0f, lardc_r_b0 = 47.0f;
float lardc_r_pwm_min = -1000.0f, lardc_r_pwm_max = 1000.0f;
float lardc_l_h = 0.001f, lardc_l_r = 150.0f, lardc_l_wc = 1.0f, lardc_l_w0 = 120.0f, lardc_l_b0 = 47.0f;
float lardc_l_pwm_min = -1000.0f, lardc_l_pwm_max = 1000.0f;

float pid_r_kp = 2.5f, pid_r_ts = 0.015f, pid_r_ki = 1.1f, pid_r_kd = 1.2f;
float pid_r_error_filter = 1.1f, pid_r_output_max = 120.0f, pid_r_output_min = -120.0f;
float pid_r_integral_max = 60.0f, pid_r_integral_min = -60.0f;

float pid_l_kp = 2.5f, pid_l_ts = 0.015f, pid_l_ki = 1.1f, pid_l_kd = 1.2f;
float pid_l_error_filter = 1.1f, pid_l_output_max = 120.0f, pid_l_output_min = -120.0f;
float pid_l_integral_max = 60.0f, pid_l_integral_min = -60.0f;

float onto_kp = 0.0f, onto_kp2 = 0.65f, onto_kd = 4.0f, onto_limit = 45.0f;

// 三串 PD 参数缓存：方向环输出目标角速度，角速度环输出左右轮速度差。
float angle_speed_kp = 0.0f,angle_speed_kd = 0.0f,angle_output_max = 700.0f,angle_output_min = -700.0f;
float onto_control_kp = 0.0f,onto_control_kd = 0.0f,angle_speed_output_max = 700.0f,angle_speed_output_min = -700.0f; 

std::string received;

/**
 * @brief 从配置文件中读取参数并设置控制器
 * @param config_file 配置文件路径
 */
void param_loading_from_file(const char* config_file) {
    std::ifstream file(config_file);
    if (!file.is_open()) {
        // 配置文件打开失败，使用默认参数
        return;
    }
    
    std::string line;
    bool use_lardc_flag = false;
    bool use_pid_flag = false;
    
    while (std::getline(file, line)) {
        // 解析 whether_use_lardc
        if (line.find("whether_use_lardc:") != std::string::npos) {
            if (line.find("true") != std::string::npos) {
                use_lardc_flag = true;
            } else {
                use_lardc_flag = false;
            }
        }
        
        // 解析 whether_use_pid
        else if (line.find("whether_use_pid:") != std::string::npos) {
            if (line.find("true") != std::string::npos) {
                use_pid_flag = true;
            } else {
                use_pid_flag = false;
            }
        }
        
        // 解析 lardc_r
        else if (line.find("lardc_r:") != std::string::npos) {
            size_t pos = line.find("h=");
            if (pos != std::string::npos) lardc_r_h = std::stof(line.substr(pos + 2));
            pos = line.find("r=");
            if (pos != std::string::npos) lardc_r_r = std::stof(line.substr(pos + 2));
            pos = line.find("wc=");
            if (pos != std::string::npos) lardc_r_wc = std::stof(line.substr(pos + 3));
            pos = line.find("w0=");
            if (pos != std::string::npos) lardc_r_w0 = std::stof(line.substr(pos + 3));
            pos = line.find("b0=");
            if (pos != std::string::npos) lardc_r_b0 = std::stof(line.substr(pos + 3));
            pos = line.find("pwm_min=");
            if (pos != std::string::npos) lardc_r_pwm_min = std::stof(line.substr(pos + 8));
            pos = line.find("pwm_max=");
            if (pos != std::string::npos) lardc_r_pwm_max = std::stof(line.substr(pos + 8));
        }
        
        // 解析 lardc_l
        else if (line.find("lardc_l:") != std::string::npos) {
            size_t pos = line.find("h=");
            if (pos != std::string::npos) lardc_l_h = std::stof(line.substr(pos + 2));
            pos = line.find("r=");
            if (pos != std::string::npos) lardc_l_r = std::stof(line.substr(pos + 2));
            pos = line.find("wc=");
            if (pos != std::string::npos) lardc_l_wc = std::stof(line.substr(pos + 3));
            pos = line.find("w0=");
            if (pos != std::string::npos) lardc_l_w0 = std::stof(line.substr(pos + 3));
            pos = line.find("b0=");
            if (pos != std::string::npos) lardc_l_b0 = std::stof(line.substr(pos + 3));
            pos = line.find("pwm_min=");
            if (pos != std::string::npos) lardc_l_pwm_min = std::stof(line.substr(pos + 8));
            pos = line.find("pwm_max=");
            if (pos != std::string::npos) lardc_l_pwm_max = std::stof(line.substr(pos + 8));
        }
        
        // 解析 pid_r
        else if (line.find("pid_r:") != std::string::npos) {
            size_t pos = line.find("p=");
            if (pos != std::string::npos) pid_r_kp = std::stof(line.substr(pos + 2));
            pos = line.find("t=");
            if (pos != std::string::npos) pid_r_ts = std::stof(line.substr(pos + 2));
            pos = line.find("i_gain=");
            if (pos != std::string::npos) pid_r_ki = std::stof(line.substr(pos + 7));
            pos = line.find("d=");
            if (pos != std::string::npos) pid_r_kd = std::stof(line.substr(pos + 2));
            pos = line.find("error_filter=");
            if (pos != std::string::npos) pid_r_error_filter = std::stof(line.substr(pos + 13));
            pos = line.find("output_max=");
            if (pos != std::string::npos) pid_r_output_max = std::stof(line.substr(pos + 11));
            pos = line.find("output_min=");
            if (pos != std::string::npos) pid_r_output_min = std::stof(line.substr(pos + 11));
            pos = line.find("integral_max=");
            if (pos != std::string::npos) pid_r_integral_max = std::stof(line.substr(pos + 13));
            pos = line.find("integral_min=");
            if (pos != std::string::npos) pid_r_integral_min = std::stof(line.substr(pos + 13));
        }
        
        // 解析 pid_l
        else if (line.find("pid_l:") != std::string::npos) {
            size_t pos = line.find("p=");
            if (pos != std::string::npos) pid_l_kp = std::stof(line.substr(pos + 2));
            pos = line.find("t=");
            if (pos != std::string::npos) pid_l_ts = std::stof(line.substr(pos + 2));
            pos = line.find("i_gain=");
            if (pos != std::string::npos) pid_l_ki = std::stof(line.substr(pos + 7));
            pos = line.find("d=");
            if (pos != std::string::npos) pid_l_kd = std::stof(line.substr(pos + 2));
            pos = line.find("error_filter=");
            if (pos != std::string::npos) pid_l_error_filter = std::stof(line.substr(pos + 13));
            pos = line.find("output_max=");
            if (pos != std::string::npos) pid_l_output_max = std::stof(line.substr(pos + 11));
            pos = line.find("output_min=");
            if (pos != std::string::npos) pid_l_output_min = std::stof(line.substr(pos + 11));
            pos = line.find("integral_max=");
            if (pos != std::string::npos) pid_l_integral_max = std::stof(line.substr(pos + 13));
            pos = line.find("integral_min=");
            if (pos != std::string::npos) pid_l_integral_min = std::stof(line.substr(pos + 13));
        }
        
        // 解析 onto_pd
        else if (line.find("onto_pd:") != std::string::npos) {
            size_t pos = line.find("kp=");
            if (pos != std::string::npos) onto_kp = std::stof(line.substr(pos + 3));
            pos = line.find("kp2=");
            if (pos != std::string::npos) onto_kp2 = std::stof(line.substr(pos + 4));
            pos = line.find("kd=");
            if (pos != std::string::npos) onto_kd = std::stof(line.substr(pos + 3));
            pos = line.find("limit=");
            if (pos != std::string::npos) onto_limit = std::stof(line.substr(pos + 6));
        }

        // 解析三串控制器的角速度环与方向环
        // 解析 angle_speed_pd
        else if (line.find("angle_speed_pd:") != std::string::npos) {
            size_t pos = line.find("kp=");
            if (pos != std::string::npos) angle_speed_kp = std::stof(line.substr(pos + 3));
            pos = line.find("kd=");
            if (pos != std::string::npos) angle_speed_kd = std::stof(line.substr(pos + 3));
        }

        // 解析 onto_control
        else if (line.find("onto_control:") != std::string::npos) {
            size_t pos = line.find("kp=");
            if (pos != std::string::npos) onto_control_kp = std::stof(line.substr(pos + 3));
            pos = line.find("kd=");
            if (pos != std::string::npos) onto_control_kd = std::stof(line.substr(pos + 3));
        }
    }
    
    file.close();
    
    // 设置 control_model
    if (use_lardc_flag && use_pid_flag) {
        control_model = 0;  // 两个都true，使用PID模式
    } else if (!use_lardc_flag && !use_pid_flag) {
        control_model = 0;  // 两个都false，使用PID模式
    } else if (use_lardc_flag && !use_pid_flag) {
        control_model = 1;  // 只有LADRC为true，使用LADRC模式
    } else {
        control_model = 0;  // 只有PID为true，使用PID模式
    }
    
    // 写入 LADRC 参数
    ladrc_right.init(lardc_r_h, lardc_r_r, lardc_r_wc, lardc_r_w0, lardc_r_b0, lardc_r_pwm_min, lardc_r_pwm_max);
    ladrc_right.setSpeedLimits(-CRUISING_SPEED, 1.5f * CRUISING_SPEED);
    
    ladrc_left.init(lardc_l_h, lardc_l_r, lardc_l_wc, lardc_l_w0, lardc_l_b0, lardc_l_pwm_min, lardc_l_pwm_max);
    ladrc_left.setSpeedLimits(-CRUISING_SPEED, 1.5f * CRUISING_SPEED);
    
    // 写入 PID 参数
    pid_r.init(pid_r_kp, pid_r_ts, pid_r_ki, pid_r_kd, pid_r_error_filter, 
               pid_r_output_max, pid_r_output_min, pid_r_integral_max, pid_r_integral_min);
    pid_l.init(pid_l_kp, pid_l_ts, pid_l_ki, pid_l_kd, pid_l_error_filter, 
               pid_l_output_max, pid_l_output_min, pid_l_integral_max, pid_l_integral_min);
    
    // 写入方向环 PD 参数
    pid_angle.setParameters(onto_kp, onto_kp2, onto_kd);
    pid_angle.setOutputLimit(onto_limit);
    onto_control_pd.init(onto_control_kp,1.0f,0.0f,onto_control_kd,5,700.0f,-700.0f,0.0f,0.0f);
    angle_speed_pd.init(angle_speed_kp,1.0f,0.0f,angle_speed_kd,10,CRUISING_SPEED*0.6,-CRUISING_SPEED*0.6,0.0f,0.0f);
}

/**
 * @brief 将参数写入控制器
 */
void param_setting(){
    // 写入 LADRC 参数
    ladrc_right.init(lardc_r_h, lardc_r_r, lardc_r_wc, lardc_r_w0, lardc_r_b0, lardc_r_pwm_min, lardc_r_pwm_max);
    ladrc_right.setSpeedLimits(-CRUISING_SPEED, 1.5f * CRUISING_SPEED);
    
    ladrc_left.init(lardc_l_h, lardc_l_r, lardc_l_wc, lardc_l_w0, lardc_l_b0, lardc_l_pwm_min, lardc_l_pwm_max);
    ladrc_left.setSpeedLimits(-CRUISING_SPEED, 1.5f * CRUISING_SPEED);
    
    // 写入 PID 参数
    pid_r.init(pid_r_kp, pid_r_ts, pid_r_ki, pid_r_kd, pid_r_error_filter, 
               pid_r_output_max, pid_r_output_min, pid_r_integral_max, pid_r_integral_min);
    pid_l.init(pid_l_kp, pid_l_ts, pid_l_ki, pid_l_kd, pid_l_error_filter, 
               pid_l_output_max, pid_l_output_min, pid_l_integral_max, pid_l_integral_min);
    
    // 写入方向环 PD 参数
    pid_angle.setParameters(onto_kp, onto_kp2, onto_kd);
    pid_angle.setOutputLimit(onto_limit);
}

bool write_param_into_file() {
    std::ofstream file("car_config.txt");
    if (!file.is_open()) {
        printf("错误：无法打开配置文件 car_config.txt 进行写入\n");
        return false;
    }

    // 写入 whether_use_lardc
    file << "whether_use_lardc: ";
    if (control_model == 1) {
        file << "true\n";
    } else {
        file << "false\n";
    }
    
    // 写入 lardc_r 参数
    file << "lardc_r: h=" << lardc_r_h << "f,r=" << lardc_r_r << "f,wc=" << lardc_r_wc << "f,w0=" << lardc_r_w0 << "f,b0=" << lardc_r_b0 << "f,pwm_min=" << lardc_r_pwm_min << "f,pwm_max=" << lardc_r_pwm_max << "f;\n";
    
    // 写入 lardc_l 参数
    file << "lardc_l: h=" << lardc_l_h << "f,r=" << lardc_l_r << "f,wc=" << lardc_l_wc << "f,w0=" << lardc_l_w0 << "f,b0=" << lardc_l_b0 << "f,pwm_min=" << lardc_l_pwm_min << "f,pwm_max=" << lardc_l_pwm_max << "f;\n";
    
    // 写入 whether_use_pid
    file << "whether_use_pid: ";
    if (control_model == 0) {
        file << "true\n";
    } else {
        file << "false\n";
    }
    
    // 写入 pid_r 参数
    file << "pid_r: p=" << pid_r_kp << "f,t=" << pid_r_ts << "f,i_gain=" << pid_r_ki << "f,d=" << pid_r_kd << "f,error_filter=" << pid_r_error_filter << "f,output_max=" << pid_r_output_max << "f,output_min=" << pid_r_output_min << "f,integral_max=" << pid_r_integral_max << "f,integral_min=" << pid_r_integral_min << "f;\n";
    
    // 写入 pid_l 参数
    file << "pid_l: p=" << pid_l_kp << "f,t=" << pid_l_ts << "f,i_gain=" << pid_l_ki << "f,d=" << pid_l_kd << "f,error_filter=" << pid_l_error_filter << "f,output_max=" << pid_l_output_max << "f,output_min=" << pid_l_output_min << "f,integral_max=" << pid_l_integral_max << "f,integral_min=" << pid_l_integral_min << "f;\n";
    
    // 写入 onto_pd 参数
    file << "onto_pd: kp=" << onto_kp << "f,kp2=" << onto_kp2 << "f,kd=" << onto_kd << "f,limit=" << onto_limit << "f;\n";
    
    // 写入 angle_speed_pd 参数
    file << "angle_speed_pd: kp=" << angle_speed_kp << "f,kd=" << angle_speed_kd << "f;\n";

    // 写入 onto_control 参数
    file << "onto_control: kp=" << onto_control_kp << "f,kd=" << onto_control_kd << "f;\n";

    // 检查写入是否成功
    if (file.fail()) {
        printf("错误：写入配置文件失败\n");
        file.close();
        return false;
    }
    
    file.close();
    printf("配置文件保存成功: car_config.txt\n");
    return true;
}

/**
 * @brief 打印当前所有控制器参数
 */
void param_print() {
    printf("\n========== 当前控制器参数 ==========\n");
    
    // 控制模式
    printf("[控制模式] ");
    if (control_model == 0) {
        printf("PID模式\n");
    } else {
        printf("LADRC模式\n");
    }
    
    // ========== LADRC 参数 ==========
    printf("\n---------- LADRC 参数 ----------\n");
    printf("左轮 (L):\n");
    printf("  h=%.3f, r=%.1f, wc=%.1f, w0=%.1f, b0=%.3f\n", 
           lardc_l_h, lardc_l_r, lardc_l_wc, lardc_l_w0, lardc_l_b0);
    printf("  pwm_min=%.0f, pwm_max=%.0f\n", lardc_l_pwm_min, lardc_l_pwm_max);
    printf("  speed_limit=[-%.1f, %.1f]\n", CRUISING_SPEED, 1.5f * CRUISING_SPEED);
    
    printf("右轮 (R):\n");
    printf("  h=%.3f, r=%.1f, wc=%.1f, w0=%.1f, b0=%.3f\n", 
           lardc_r_h, lardc_r_r, lardc_r_wc, lardc_r_w0, lardc_r_b0);
    printf("  pwm_min=%.0f, pwm_max=%.0f\n", lardc_r_pwm_min, lardc_r_pwm_max);
    printf("  speed_limit=[-%.1f, %.1f]\n", CRUISING_SPEED, 1.5f * CRUISING_SPEED);
    
    // ========== PID 参数 ==========
    printf("\n---------- PID 参数 ----------\n");
    printf("左轮 (L):\n");
    printf("  Kp=%.2f, Ki=%.2f, Kd=%.2f, Ts=%.3f\n", 
           pid_l_kp, pid_l_ki, pid_l_kd, pid_l_ts);
    printf("  error_filter=%.2f\n", pid_l_error_filter);
    printf("  output_limit=[%.0f, %.0f]\n", pid_l_output_min, pid_l_output_max);
    printf("  integral_limit=[%.0f, %.0f]\n", pid_l_integral_min, pid_l_integral_max);
    
    printf("右轮 (R):\n");
    printf("  Kp=%.2f, Ki=%.2f, Kd=%.2f, Ts=%.3f\n", 
           pid_r_kp, pid_r_ki, pid_r_kd, pid_r_ts);
    printf("  error_filter=%.2f\n", pid_r_error_filter);
    printf("  output_limit=[%.0f, %.0f]\n", pid_r_output_min, pid_r_output_max);
    printf("  integral_limit=[%.0f, %.0f]\n", pid_r_integral_min, pid_r_integral_max);
    
    // ========== 方向环 PD 参数 ==========
    printf("\n---------- 方向环 PD 参数 ----------\n");
    printf("  Kp=%.3f, Kp2=%.3f, Kd=%.2f\n", onto_kp, onto_kp2, onto_kd);
    printf("  output_limit=%.1f\n", onto_limit);
    
    // ========== 系统常量 ==========
    printf("\n---------- 系统常量 ----------\n");
    printf("  CRUISING_SPEED = %.1f m/s\n", CRUISING_SPEED);
    printf("  编码器采样周期 = %d ms\n", ENCODER_SAMPLING_PERIOD);
    printf("  PID控制周期 = %d ms\n", PID_CONTROL_PERIOD);
    printf("  LADRC控制周期 = %d ms\n", LARDC_PERIOD);
    printf("  按键扫描周期 = %d ms\n", KEY_SCAN_PERIOD);
    
        // ========== 三串 PD 参数 ==========
    printf("\n---------- 三串 PD 参数 ----------\n");
    printf("  angle_speed_pd: Kp=%.3f, Kd=%.3f\n", angle_speed_kp, angle_speed_kd);
    printf("  onto_control: Kp=%.3f, Kd=%.3f\n", onto_control_kp, onto_control_kd);

    printf("\n===================================\n");
    fflush(stdout);
}

/**
 * @brief 处理 READ 命令：从文件重新加载参数并更新控制器
 */
void handleReadCommand() {
    param_loading_from_file("/home/root/car_config.txt");
    param_setting();
    printf("已从文件重新加载参数\n");
    param_print();
}

/**
 * @brief 处理 WRITE 命令：将当前参数发送回去
 */
bool handleWriteCommand(TCPClient& client) {
    if(write_param_into_file()){
        client.sendFormattedData("b0:%.6f\n", lardc_r_b0);
        client.sendFormattedData("wc:%.6f\n", lardc_r_wc);
        client.sendFormattedData("w0:%.6f\n", lardc_r_w0);
        client.sendFormattedData("r:%.6f\n", lardc_r_r);
        client.sendFormattedData("h:%.6f\n", lardc_r_h);
        client.sendFormattedData("kp:%.6f\n", onto_kp);
        client.sendFormattedData("kp2:%.6f\n", onto_kp2);
        client.sendFormattedData("kd:%.6f\n", onto_kd);
        client.sendFormattedData("limit:%.6f\n", onto_limit);
        client.sendFormattedData("angle_speed_kp:%.6f\n", angle_speed_kp);
        client.sendFormattedData("angle_speed_kd:%.6f\n", angle_speed_kd);
        client.sendFormattedData("onto_control_kp:%.6f\n", onto_control_kp);
        client.sendFormattedData("onto_control_kd:%.6f\n", onto_control_kd);
        printf("已发送当前参数\n");
        return true;
    }
    return false;
}

/**
 * @brief 解析接收到的参数并更新对应的全局变量
 * @param line 接收到的数据行（格式如 "b0:4.000000"）
 * @return true 解析成功并更新，false 解析失败
 */
bool parseAndUpdateParameter(const std::string& line) {
    // 查找冒号分隔符
    size_t colon_pos = line.find(':');
    if (colon_pos == std::string::npos) {
        return false;
    }
    std::string key = line.substr(0, colon_pos);
    std::string value_str = line.substr(colon_pos + 1);
    
    // 去除可能的空格和换行符
    key.erase(0, key.find_first_not_of(" \t\r\n"));
    key.erase(key.find_last_not_of(" \t\r\n") + 1);
    value_str.erase(0, value_str.find_first_not_of(" \t\r\n"));
    value_str.erase(value_str.find_last_not_of(" \t\r\n") + 1);
    
    float value = std::stof(value_str);
    
    // 根据 key 更新对应的参数
    if (key == "b0") {
        lardc_r_b0 = value;
        lardc_l_b0 = value;
        printf("更新 b0: %.6f\n", value);
    }
    else if (key == "wc") {
        lardc_r_wc = value;
        lardc_l_wc = value;
        printf("更新 wc: %.6f\n", value);
    }
    else if (key == "w0") {
        lardc_r_w0 = value;
        lardc_l_w0 = value;
        printf("更新 w0: %.6f\n", value);
    }
    else if (key == "r") {
        lardc_r_r = value;
        lardc_l_r = value;
        printf("更新 r: %.6f\n", value);
    }
    else if (key == "h") {
        lardc_r_h = value;
        lardc_l_h = value;
        printf("更新 h: %.6f\n", value);
    }
    else if (key == "kp") {
        onto_kp = value;
        printf("更新 onto_kp: %.6f\n", value);
    }
    else if (key == "kp2") {
        onto_kp2 = value;
        printf("更新 onto_kp2: %.6f\n", value);
    }
    else if (key == "kd") {
        onto_kd = value;
        printf("更新 onto_kd: %.6f\n", value);
    }
    else if (key == "limit") {
        onto_limit = value;
        printf("更新 onto_limit: %.6f\n", value);
    }
    else if (key == "pid_kp") {
        pid_r_kp = value;
        pid_l_kp = value;
        printf("更新 PID Kp: %.6f\n", value);
    }
    else if (key == "pid_ki") {
        pid_r_ki = value;
        pid_l_ki = value;
        printf("更新 PID Ki: %.6f\n", value);
    }
    else if (key == "pid_kd") {
        pid_r_kd = value;
        pid_l_kd = value;
        printf("更新 PID Kd: %.6f\n", value);
    }else if (key == "angle_speed_kp") {
    angle_speed_kp = value;
    printf("更新 angle_speed_kp: %.6f\n", value);
    }
    else if (key == "angle_speed_kd") {
        angle_speed_kd = value;
        printf("更新 angle_speed_kd: %.6f\n", value);
    }
    else if (key == "onto_control_kp") {
        onto_control_kp = value;
        printf("更新 onto_control_kp: %.6f\n", value);
    }
    else if (key == "onto_control_kd") {
        onto_control_kd = value;
        printf("更新 onto_control_kd: %.6f\n", value);
    }else {
        printf("未知参数: %s\n", key.c_str());
        return false;
    }


    
    return true;
}
