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
    printf("whether get image:%d\n", uvc.wait_image_refresh());
    ips200.show_gray_image(10, 10, gray_img_ptr, UVC_WIDTH, UVC_HEIGHT);
    //编码器及电机速度测试
    motor_set_speed(0,1200);
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
    // motor_set_speed(0,0);
    //PID控制器控速测试
    target_speed_r = 20;
    target_speed_l = -20;
    printf("right_speed: %4.2f ,left_speed: %4.2f\r", right_speed, left_speed);
}

void send_picture_to_Serve()
{
    rgb_img_ptr = uvc.get_rgb_image_ptr();
    uvc.wait_image_refresh();
    rgb_img_transmitter(rgb_img_ptr, UVC_WIDTH, UVC_HEIGHT,true);
}

void tracking()
{
    //图像获取与处理
    image_proc();
    
    // 传输灰度图像 + 三条边线（左边线、右边线、中线）
    // 参数说明：
    //   - sampled_Lline, sampled_Lline_num: 左边线坐标和点数（蓝色）
    //   - sampled_Rline, sampled_Rline_num: 右边线坐标和点数（红色）
    //   - Mline, Mline_num: 中线坐标和点数（绿色）
    //   - true, true: 垂直翻转和水平翻转（根据你的摄像头安装方向调整）
    gray_img_with_centerline_transmitter(img_gray, UVC_WIDTH, IMG_H, 
                                        sampled_Lline, sampled_Lline_num,
                                        sampled_Rline, sampled_Rline_num,
                                        Mline, Mline_num,
                                        true, true);
    // rgb_img_transmitter(reinterpret_cast<const uint16_t*>(uvc.frame_rgb.ptr()), UVC_WIDTH, UVC_HEIGHT,true);
}

