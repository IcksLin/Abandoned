/*********************************************************************************************************************
* 文件名称          my_task_function
* 功能说明          主循环任务函数实现
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone          first version
* 2026-05-01        Assistant                  补充 Doxygen 注释
********************************************************************************************************************/



#include "my_task_function.hpp"
// 注意，所有组件的声明全在 global 文件中

/**
 * @brief 定时线程回调测试函数
 * @note 每调用一次递增并打印计数，用于确认线程是否正常运行。
 */
void thread_callback_test(){
    static int32_t callback_function_tick = 0;
    callback_function_tick++;
    printf("callback success :%d\n",callback_function_tick);
}

/**
 * @brief 摄像头灰度图显示测试函数
 */
void camera_test(){
    gray_img_ptr = uvc.get_gray_image_ptr();
    printf("whether get image:%d\n", uvc.wait_image_refresh());
    ips200.show_gray_image(10, 10, gray_img_ptr, UVC_WIDTH, UVC_HEIGHT);

}

/**
 * @brief 电机速度开环测试函数
 * @param speed_left 左轮速度指令
 * @param speed_right 右轮速度指令
 */
void motor_speed_test(int16_t speed_left,int16_t speed_right){
    motor_set_speed(speed_left,speed_right);
    get_and_remap_speed(&right_speed,&left_speed,10);
}

/**
 * @brief 无屏幕综合状态测试
 * @note 一旦使用 IPS 屏幕，将会严重打乱线程周期。该函数仅作测试所有模块是否正常运行用。
 */
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

/**
 * @brief 将当前 RGB 图像发送到上位机
 */
void send_picture_to_Serve()
{
    rgb_img_ptr = uvc.get_rgb_image_ptr();
    uvc.wait_image_refresh();
    rgb_img_transmitter(rgb_img_ptr, UVC_WIDTH, UVC_HEIGHT,true);
}

/**
 * @brief 主循迹任务
 * @details 等待摄像头刷新，复制 RGB 帧并调用 image_proc 完成图像处理。
 */
void tracking()
{
    // my_timer.stop();
    // printf("time: %lld  ms",my_timer.elapsed_ms());

    //图像采样
    uvc.wait_image_refresh();
    uvc.frame_rgb = uvc.frame_mjpg.clone();

    image_proc();

    // cruising_speed = 0;
    
    // 开关，1使能循迹
    // onto_pd_control_enable = 0;
    // my_timer.start();
    // 调试信息===========================================
    // printf("   L: %f   ,R:  %f    \r",nms_Lline, nms_Rline);
}

/**
 * @brief 数据集采集辅助函数
 * @note 只等待并获取 RGB 图像，不做图像处理。
 */
void get_image_datasets(){
    uvc.wait_image_refresh();
    uvc.get_rgb_image_ptr();
}
