/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/



#include "my_task_function.hpp"
//注意，所有组件的声明全在globel文件中

void thread_callback_test(){
    static int32_t callback_function_tick = 0;
    callback_function_tick++;
    printf("callback success :%d\n",callback_function_tick);
}

//camera==================================================
void camera_test(){
    gray_img_ptr = uvc.get_gray_image_ptr();
    printf("whether get image:%d\n", uvc.wait_image_refresh());
    ips200.show_gray_image(10, 10, gray_img_ptr, UVC_WIDTH, UVC_HEIGHT);

}

//motor===================================================
void motor_speed_test(int16_t speed_left,int16_t speed_right){
    motor_set_speed(speed_left,speed_right);
    get_and_remap_speed(&right_speed,&left_speed,10);
}

//show test===============================================
//注意，一旦使用ips屏幕，将会严重打乱线程周期，故此函数仅作测试所有模块是否正常运行用，实际测量编码器，PID等请用无显示屏测试函数
void show_all_of_the_component(){
    //图像测试
    gray_img_ptr = uvc.get_gray_image_ptr();
    // printf("whether get image:%d\n", uvc.wait_image_refresh());
    ips200.show_gray_image(10, 10, gray_img_ptr, UVC_WIDTH, UVC_HEIGHT);
    //编码器及电机速度测试
    // motor_set_speed(0,1200);
    ips200.show_string(10,140,"right_speed:");
    ips200.show_string(10,156,"left_speed :");
    ips200.show_float(50,140,right_speed,4,2);
    ips200.show_float(50,156,left_speed ,4,2);
}

void show_all_of_the_component_without_ips(){
    
    //图像测试
    // camera_timer.start();
    gray_img_ptr = uvc.get_gray_image_ptr();
    uvc.wait_image_refresh();
    // printf("whether get image:%d\n", uvc.wait_image_refresh());
    // camera_timer.stop();
    // printf("camera speed timer: %d\n",camera_timer.elapsed_ms());
    //编码器及电机速度测试
    motor_set_speed(40,40);
    //PID控制器控速测试
    // target_speed_r = 0;
    // target_speed_l = 0;

    onto_pd_control_enable = 0;

    printf("right_speed: %4.2f ,left_speed: %4.2f,pitch: %4.2f,roll: %4.2f,yall: %4.2f    \r",
         right_speed, left_speed,ahrs.getPitch(),ahrs.getRoll(),ahrs.getYaw());
}

void send_picture_to_Serve()
{
    rgb_img_ptr = uvc.get_rgb_image_ptr();
    uvc.wait_image_refresh();
    rgb_img_transmitter(rgb_img_ptr, UVC_WIDTH, UVC_HEIGHT,true);
}

/**
 * 集成控制函数
 * 逻辑：检测按键则加速，无按键则自动减速至0
 * @param dt 两次调用之间的时间间隔（秒），用于保证物理曲线平滑
 */
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <algorithm>
void updateDriveControl() {
    static bool initialized = false;
    if (!initialized) {
        struct termios newt;
        tcgetattr(STDIN_FILENO, &newt);
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        initialized = true;
    }

    // --- 动力参数配置 ---
    const float MAX_SPEED   = 30.0f;
    const float RESPONSIVENESS = 29.9f; // 加速灵敏度
    const float STOP_SHARPNESS = 29.9f; // 减速灵敏度（刹车更快）
    const float DEADZONE       = 0.1f;

    // 状态位
    bool up_pressed    = false;
    bool left_pressed  = false;
    bool right_pressed = false;

    // --- 1. 读取并清空缓冲区 (获取最新按键) ---
    unsigned char buf[3];
    while (read(STDIN_FILENO, buf, 3) > 0) {
        if (buf[0] == 0x1B && buf[1] == 0x5B) {
            switch(buf[2]) {
                case 'A': up_pressed = true;    break; // Up Arrow
                case 'D': left_pressed = true;  break; // Left Arrow
                case 'C': right_pressed = true; break; // Right Arrow
            }
        }
    }

    // --- 2. 目标速度逻辑分配 ---
    float goal_l = 0.0f;
    float goal_r = 0.0f;

    if (up_pressed) {
        goal_l = MAX_SPEED;
        goal_r = MAX_SPEED;
    } else if (left_pressed) {
        goal_l = 0.0f;
        goal_r = MAX_SPEED; 
    } else if (right_pressed) {
        goal_l = MAX_SPEED;
        goal_r = 0.0f; 
    }

    auto apply_curve = [&](float& current, float goal) {
        float factor = (goal > std::abs(current) + 0.1f) ? RESPONSIVENESS : STOP_SHARPNESS;
        current += (goal - current) * factor;
        
        // 死区处理
        if (std::abs(current) < DEADZONE) current = 0.0f;
        
        // 在函数内部进行物理限幅
        current = std::clamp(current, 0.0f, MAX_SPEED);
    };

    apply_curve(target_speed_l, goal_l);
    apply_curve(target_speed_r, goal_r);

}

void tracking()
{
    //图像获取与处理
    // printf("max_angle L: %.2f at idx %d, R: %.2f at idx %d, onto: %.2f,time: %lld ms,max_corner: %.2f       \n", 
    //     nms_Lline, nms_Lline_idx, nms_Rline,
    //      nms_Rline_idx, onto
    //      , my_timer.elapsed_ms(), max_angle);
    // my_timer.stop();
    // printf("time: %lld  ms",my_timer.elapsed_ms());

    //图像采样
    uvc.wait_image_refresh();
    uvc.frame_rgb = uvc.frame_mjpg.clone();

    image_proc();
    // 龙邱----------------------------------------------------------------------------------------------------------------------
    // if(red_detector.model_roi_cut(uvc.frame_rgb, red_detector.target_roi, true)) { 
    //     // 1. 计算红框中心点
    //     int cx = red_detector.target_rect.x + red_detector.target_rect.width / 2;
    //     int cy = red_detector.target_rect.y + red_detector.target_rect.height / 2;

    //     // 2. 计算允许通行的像素边界 (基于 160x120)
    //     int x_min = UVC_WIDTH * limit_left;
    //     int x_max = UVC_WIDTH * (1.0f - limit_right);
    //     int y_min = UVC_HEIGHT * limit_up;
    //     int y_max = UVC_HEIGHT * (1.0f - limit_down);

    //     // 3. 位置限制判定
    //     if (cx >= x_min && cx <= x_max && cy >= y_min && cy <= y_max) {
            
    //         // --- [NCNN 执行推理] ---
    //         my_timer.start();
    //         LQ_InferenceResult result = ncnn_classifier.infer(red_detector.target_roi);
    //         my_timer.stop();

    //         // 4. 置信度过滤与类别计入
    //         std::string final_label = "None";
    //         float final_conf = 0.0f;

    //         // 只有高于阈值的才被计入有效票数
    //         if (result.confidence >= CONFIDENCE_THRESHOLD) {
    //             final_label = result.label;
    //             final_conf = result.confidence;
    //         } else {
    //             final_label = "None";
    //             final_conf = 0.0f; // 低置信度帧权重为 0
    //         }

    //         // 5. 累计数据
    //         confidence_accumulator[final_label] += final_conf;
    //         frame_count++;

    //         // 6. 达到 MAX_ACCUMULATE_FRAMES (25次) 后计算结果
    //         if (frame_count >= MAX_ACCUMULATE_FRAMES) {
    //             std::string best_class = "None";
    //             float max_avg_conf = -1.0f;

    //             for (auto const& [label, total_conf] : confidence_accumulator) {
    //                 float avg = total_conf / MAX_ACCUMULATE_FRAMES;
    //                 if (avg > max_avg_conf) {
    //                     max_avg_conf = avg;
    //                     best_class = label;
    //                 }
    //             }

    //             // 静态显示最终统计结果
    //             printf("\n========================================\n");
    //             printf(">>> [NCNN 25帧统计结果]\n");
    //             printf("    最终类别: %s\n", best_class.c_str());
    //             printf("    平均置信度: %.2f%%\n", max_avg_conf * 100);
    //             printf("    平均单帧耗时: %lldms\n", my_timer.elapsed_ms());
    //             printf("========================================\n");

    //             // 清空累计器，准备下一轮统计
    //             confidence_accumulator.clear();
    //             frame_count = 0;
    //         } else {
    //             // 显示累计进度
    //             printf("\r[NCNN 累计 %d/%d] 实时: %s (%.2f%%) | 中心: (%d,%d)    ", 
    //                 frame_count, MAX_ACCUMULATE_FRAMES, 
    //                 result.label.c_str(), result.confidence * 100,
    //                 cx, cy);
    //             fflush(stdout);
    //         }

    //     } else {
    //         // --- [位置超出限制范围] ---
    //         if (frame_count > 0) {
    //             confidence_accumulator.clear();
    //             frame_count = 0;
    //             printf("\n[WARN] 目标漂出中心区，累计已重置\n");
    //         }
    //         printf("\r[区域外] 中心点: (%3d,%3d) | 限制: X[%d-%d] Y[%d-%d]    ", 
    //             cx, cy, x_min, x_max, y_min, y_max);
    //         fflush(stdout);
    //     }

    // } else {
    //     // --- [未检测到目标红框] ---
    //     if (frame_count > 0) {
    //         confidence_accumulator.clear();
    //         frame_count = 0;
    //         printf("\n[WARN] 目标丢失，累计已重置\n");
    //     }
    //     printf("\r[等待] 画面中未发现红色目标...                        ");
    //     fflush(stdout);
    // }
    //end------------------------------------------------------------------------------------------------------------

    cruising_speed = std::max<float>(40.0f, (float)CRUISING_SPEED - (std::abs(onto) / 30.0f) * ((float)CRUISING_SPEED * 0.2f) - (1.0f - std::clamp((float)(middle_line_length - 45) / 45.0f, 0.0f, 1.0f)) * ((float)CRUISING_SPEED * 0.25f));
    pid_angle.setOutputLimit(cruising_speed*0.40);

    // cruising_speed = speed_decision(middle_line_length,CRUISING_SPEED*0.6,CRUISING_SPEED*1.2);
    // printf("midle_line_length: %d   ,speed:%f  \r   ",middle_line_length,cruising_speed);

    
    // 传输灰度图像 + 三条边线（左边线、右边线、中线）
    // 参数说明：
    //   - Lline, Lline_num: 原始左边线坐标和点数（蓝色）
    //   - Rline, Rline_num: 原始右边线坐标和点数（红色）
    //   - Mline, Mline_num: 中线坐标和点数（绿色）
    //   - true, true: 垂直翻转和水平翻转（根据你的摄像头安装方向调整）
    // 注意：Lline和Rline是int类型，需要reinterpret_cast转换为float类型
    // static uint8_t L_buf[IMG_H][2];
    // static uint8_t R_buf[IMG_H][2];
    // static uint8_t M_buf[IMG_H][2];
    // // std::cout << "DEBUG: L_num=" << Lline_num << " R_num=" << Rline_num << std::endl;

    // for (int i = 0; i < IMG_H; ++i) {
    //     // 只有在 i 小于有效点数时才进行转换和打印
    //     if (i < sampled_Lline_num) {
    //         L_buf[i][0] = (uint8_t)std::max(0, std::min((int)sampled_Lline[i][0], 255));
    //         L_buf[i][1] = (uint8_t)std::max(0, std::min((int)sampled_Lline[i][1], 255));
            
    //         // 只打印有效点
    //         // std::cout << "Lline[" << i << "]: (" << (int)L_buf[i][0] << ", " << (int)L_buf[i][1] << ")" << std::endl;
    //     }

    //     if (i < sampled_Rline_num) {
    //         R_buf[i][0] = (uint8_t)std::max(0, std::min((int)sampled_Rline[i][0], 255));
    //         R_buf[i][1] = (uint8_t)std::max(0, std::min((int)sampled_Rline[i][1], 255));
    //         // std::cout << "Rline[" << i << "]: (" << (int)R_buf[i][0] << ", " << (int)R_buf[i][1] << ")" << std::endl;
    //     }

    //    if (i < middle_line_length) {
    //         M_buf[i][0] = (uint8_t)std::clamp((int)Mline[i][0], 0, 255);
    //         M_buf[i][1] = (uint8_t)std::clamp((int)Mline[i][1], 0, 255);
    //     }
    // }

    // udp.process_frame(uvc.frame_rgb);
    
    //方向控制器启动（PD运算在线程中执行）
    onto_pd_control_enable = 1;
    // my_timer.start();

    // 调试信息===========================================
    // printf("   L: %f   ,R:  %f    \r",nms_Lline, nms_Rline);
}

//采集数据集时使用
void get_image_datasets(){
    uvc.wait_image_refresh();
    uvc.get_rgb_image_ptr();
    
    udp.set_send_mode(MODE_COLOR);
    udp.process_frame(uvc.frame_rgb);
}