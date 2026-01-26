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
    img_ptr = uvc.get_gray_image_ptr();
    printf("whether get image:%d\n", uvc.wait_image_refresh());
    ips200.show_gray_image(10, 10, img_ptr, UVC_WIDTH, UVC_HEIGHT);

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
    img_ptr = uvc.get_gray_image_ptr();
    printf("whether get image:%d\n", uvc.wait_image_refresh());
    ips200.show_gray_image(10, 10, img_ptr, UVC_WIDTH, UVC_HEIGHT);
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
    img_ptr = uvc.get_gray_image_ptr();
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

int32_t send_picture_to_Serve()
{
    static uint32_t frame_count = 0;  // 静态变量记录帧数
    static bool is_first_call = true; // 首次调用标志
    static TimerClockGetTime send_timer; // 发送计时器
    
    // 首次调用时初始化计时器
    if (is_first_call)
    {
        send_timer.start();
        is_first_call = false;
        printf("HTTP流媒体服务器模式运行中...\n");
    }
    
    // 1. 等待并获取摄像头图像
    if (uvc.wait_image_refresh() != 0)
    {
        // printf("等待摄像头图像刷新失败\n");
        return -1;
    }
    
    img_ptr = uvc.get_gray_image_ptr();
    if (img_ptr == nullptr)
    {
        printf("获取灰度图像指针失败\n");
        return -1;
    }
    
    // 2. 更新HTTP服务器帧数据
    send_timer.start();  // 开始计时
    
    // 构造cv::Mat但不拷贝数据（引用zf_device_uvc内部缓存）
    cv::Mat frame(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, img_ptr);
    camera_server.update_frame(frame);
    
    send_timer.stop();   // 结束计时
    
    frame_count++;
    
    // 3. 打印统计信息（每60帧）
    if (frame_count % 60 == 0)
    {
        float fps = 1000.0f / (send_timer.elapsed_ms() + 1); // 避免除0
        printf("[帧号:%04u]推流更新耗时: %dms | 估算FPS: %.1f\n",
                frame_count, send_timer.elapsed_ms(), fps);
    }
    
    // 4. 在本地显示屏上显示图像
    ips200.show_gray_image(10, 10, img_ptr, UVC_WIDTH, UVC_HEIGHT);
    
    return 1; // 返回正数表示成功
}