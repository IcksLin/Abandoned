/*********************************************************************************************************************
* 文件名称          my_global
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone         
********************************************************************************************************************/


#ifndef __MY_GLOBAL_HPP__
#define __MY_GLOBAL_HPP__

#include "zf_common_headfile.hpp"

//编码器API
#define ENCODER_SAMPLING_PERIOD 3           //采样周期（ms）
extern zf_driver_encoder encoder1;
extern zf_driver_encoder encoder2;
//显示屏组件（有大药的东西，别乱用！！！）
extern zf_device_ips200 ips200;
//菜单组件
#define KEY_SCAN_PERIOD 10                   //按键扫描周期（ms）
extern MyKey key_manager;
extern MyMenu menu_system;
//电机速度API
#define PID_CONTROL_PERIOD  10*ENCODER_SAMPLING_PERIOD  //PID控制器控制周期,默认编码器采样周期5~10倍,必须大于2


extern int16_t speed_to_pwm_r;//直接写入pwm设置的值，PID的handle函数将以一定周期修改这两个值并将其填入设置函数中
extern int16_t speed_to_pwm_l;
//PID控制器
extern MyPID pid_r;  // 右轮PID
extern MyPID pid_l;  // 左轮PID

//摄像头组件
extern zf_device_uvc uvc;
extern uint8_t* img_ptr;

//时间戳
extern TimerClockGetTime my_timer;
extern TimerClockGetTime camera_timer;

//TCP图像传输
// 电脑服务器配置
#define SERVER_IP "192.168.43.4"
#define SERVER_PORT 8086
extern zf_driver_image_client img_client;

//控制接口
//将你想要设置的左右轮速度直接填入这两个变量，PID控制器会直接控制轮胎转速
extern float target_speed_r;//右轮目标速度，修改该数值可直接影响PID控制器输出
extern float target_speed_l;//左轮目标速度，修改该数值可直接影响PID控制器输出
//访问这两个值就能读取到当前左右轮胎的转速
//采样周期3ms，采集并记录3ms内编码器脉冲计数，然后清空计数
extern float right_speed;   //右轮当前速度
extern float left_speed ;   //左轮当前速度


// 声明函数
void key_scan_handler();
void encoder_get_count_handler();
void pid_contol_handle();
#endif