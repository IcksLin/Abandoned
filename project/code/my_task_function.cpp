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

void tracking()
{
    // my_timer.stop();
    // printf("time: %lld  ms",my_timer.elapsed_ms());

    //图像采样
    uvc.wait_image_refresh();
    uvc.frame_rgb = uvc.frame_mjpg.clone();

    image_proc();

    cruising_speed = 0;
    
    // 开关，1使能循迹
    onto_pd_control_enable = 0;
    // my_timer.start();
    // 调试信息===========================================
    // printf("   L: %f   ,R:  %f    \r",nms_Lline, nms_Rline);
}

//采集数据集时使用
void get_image_datasets(){
    uvc.wait_image_refresh();
    uvc.get_rgb_image_ptr();
}

