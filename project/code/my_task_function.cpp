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
    }
    
    // 1. 检查TCP连接状态
    static bool connection_warning_shown = false;
    if (!connection_warning_shown)
    {
        printf("图像传输客户端已初始化，目标: %s:%d\n", SERVER_IP, SERVER_PORT);
        connection_warning_shown = true;
    }
    
    // 2. 等待并获取摄像头图像
    if (uvc.wait_image_refresh() != 0)
    {
        printf("等待摄像头图像刷新失败\n");
        return -1;
    }
    
    img_ptr = uvc.get_gray_image_ptr();
    if (img_ptr == nullptr)
    {
        printf("获取灰度图像指针失败\n");
        return -1;
    }
    
    // 3. 发送灰度图像到服务器
    send_timer.start();  // 开始发送计时
    int32_t sent_bytes = img_client.send_gray_raw(img_ptr, UVC_WIDTH, UVC_HEIGHT, 1000);
    send_timer.stop();   // 结束发送计时
    
    frame_count++;
    
    // 4. 处理发送结果
    if (sent_bytes > 0)
    {
        // 成功发送
        if (frame_count % 10 == 0)  // 每10帧显示一次详细信息
        {
            float fps = 1000.0f / send_timer.elapsed_ms();  // 计算本次发送的瞬时FPS
            
            printf("[帧号:%04u] 发送成功: %u字节 | 尺寸: %ux%u | 耗时: %dms | FPS: %.1f\n",
                   frame_count, sent_bytes, UVC_WIDTH, UVC_HEIGHT, 
                   send_timer.elapsed_ms(), fps);
        }
        else
        {
            // 普通帧显示简单信息
            printf("[%04u] ✓ %u字节\n", frame_count, sent_bytes);
        }
    }
    else
    {
        // 发送失败
        printf("[帧号:%04u] 发送失败\n", frame_count);
        
        // 可以考虑在这里添加重连逻辑
        // if (should_reconnect) {
        //     printf("尝试重新连接服务器...\n");
        //     img_client.init(SERVER_IP, SERVER_PORT);
        // }
    }
    
    // 5. 可选：在本地显示屏上显示图像（如果需要）
    ips200.show_gray_image(10, 10, img_ptr, UVC_WIDTH, UVC_HEIGHT);
    
    return sent_bytes;
}