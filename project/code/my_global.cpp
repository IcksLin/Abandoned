/*********************************************************************************************************************
* 文件名称          my_global
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/

#include "my_global.hpp"

zf_device_ips200 ips200;
MyKey key_manager;
MyMenu menu_system(&key_manager, &ips200);
//TCP图像传输组件
zf_driver_image_client img_client;
//程序耗时检验时间戳
TimerClockGetTime my_timer;
TimerClockGetTime camera_timer;
//底层组件========================================
//变量定义

// DRV驱动相关变量,直接从这两个变量中读取速度
float right_speed = 0;
float left_speed  = 0;

//摄像头组建
zf_device_uvc uvc;
uint8_t* img_ptr = nullptr;

//上层控制组件=====================================
//PID速度控制器
MyPID pid_r;  // 右轮PID
MyPID pid_l;  // 左轮PID
float target_speed_r;//右轮目标速度，修改该数值可直接影响PID控制器输出
float target_speed_l;//左轮目标速度，修改该数值可直接影响PID控制器输出
int16_t speed_to_pwm_r;//直接写入pwm设置的值，PID的handle函数将以一定周期修改这两个值并将其填入设置函数中
int16_t speed_to_pwm_l;

void key_scan_handler() {
    key_manager.scan_keys();
}

//编码器任务进程包装函数
void encoder_get_count_handler(){

    //启动时间戳检测时间a
    // my_timer.stop();
    get_and_remap_speed(&right_speed,&left_speed,ENCODER_SAMPLING_PERIOD);
    // printf("spending time:%lld\n",my_timer.elapsed_ms());
    // my_timer.start();
}

//PID控制器进程包装函数，该进程需要严格搭配编码器进程使用
//此函数默认在该线程一启动就默认调用，若需要电机不转动，需要将目标速度设置为0
//想修改速度直接访问target_speed_r&target_speed_l即可
void pid_contol_handle(){
    speed_to_pwm_r = (int16_t)pid_r.control(target_speed_r,right_speed);
    speed_to_pwm_l = (int16_t)pid_l.control(target_speed_l,left_speed );
    motor_set_speed(speed_to_pwm_l,speed_to_pwm_r);
}