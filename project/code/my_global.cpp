/*********************************************************************************************************************
* 文件名称          my_global.cpp
* 功能说明          全局资源管理模块实现 - 全局对象定义和线程回调函数实现
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-22        HeavenCornerstone          first version
* 2026-01-25        Assistant                  添加详细注释和分区优化
********************************************************************************************************************/

#include "my_global.hpp"

/* ================================================================================================================
 *                                           人机交互设备对象定义
 * ================================================================================================================ */
// 显示屏对象
zf_device_ips200 ips200;                                        // IPS200屏幕对象，240x320分辨率

// 按键和菜单系统
MyKey key_manager;                                              // 4按键管理器
MyMenu menu_system(&key_manager, &ips200);                      // 菜单系统（关联按键和屏幕）

/* ================================================================================================================
 *                                           图像采集与传输对象定义
 * ================================================================================================================ */
// USB摄像头
zf_device_uvc uvc;                                              // UVC摄像头对象，160x120@60fps
uint8_t* gray_img_ptr = nullptr;                                // 灰度图像数据指针（备用）
uint16_t* rgb_img_ptr = nullptr;                                // RGB图像数据指针（备用）

/* ================================================================================================================
 *                                           性能分析工具对象定义
 * ================================================================================================================ */
// 时间戳对象（用于性能测试）
TimerClockGetTime my_timer;                                     // 通用计时器（纳秒级精度）
TimerClockGetTime camera_timer;                                 // 摄像头帧率计时器

/* ================================================================================================================
 *                                           速度控制系统变量定义
 * ================================================================================================================ */

// -------------------- 速度反馈变量（由编码器线程更新） --------------------
float right_speed = 0.0f;                                       // 右轮当前速度（只读）
float left_speed  = 0.0f;                                       // 左轮当前速度（只读）
                                                                // 更新周期：3ms（ENCODER_SAMPLING_PERIOD）

// -------------------- 速度设定变量（用户控制接口） --------------------
float target_speed_r = 0.0f;                                    // 右轮目标速度（可写）
float target_speed_l = 0.0f;                                    // 左轮目标速度（可写）
                                                                // 💡使用方式：直接赋值即可控制速度
                                                                // 示例：target_speed_r = 50.0f;
float cruising_speed = CRUISING_SPEED;                                   // 巡航速度，修改该值会直接影响小车速度

// -------------------- PWM输出变量（系统内部使用） --------------------
int16_t speed_to_pwm_r = 0;                                     // 右轮PWM输出值（PID计算结果）
int16_t speed_to_pwm_l = 0;                                     // 左轮PWM输出值（PID计算结果）

// -------------------- PID控制器对象 --------------------
MyPID pid_r;                                                    // 右轮PID控制器
MyPID pid_l;                                                    // 左轮PID控制器
PDController pid_angle;                                                // 角度PID控制器

uint8_t onto_pd_control_enable = 0;                          // 角度PD控制使能标志（0=禁用，1=启用）

//---------------------- IMU ---------------------------
IMUHandler imu963r;                                             //陀螺仪与按键扫描在同一线程中执行，均为10ms
MadgwickAHRS ahrs(100);                                         //解算器，同步获取同步处理，注意检查性能消耗
float IMU_calibration = 180.0f/154.03f;                         //范围校准

//--------------------惯性导航控制器-----------------------------------
PathTracker path_tracker_component;                             //路径记录组件，内置里程计
AkimaInterpolator akima_component;                              //地图解算组件，将地图存储为指定名字的txt文件，格式：索引 x y

/* ================================================================================================================
 *                                           线程回调函数实现
 * ================================================================================================================ */

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     按键扫描线程回调函数
// 参数说明     无
// 返回参数     void
// 调用周期     KEY_SCAN_PERIOD (10ms)
// 线程优先级   98
// 备注信息     执行按键状态机扫描，检测按键事件
//-------------------------------------------------------------------------------------------------------------------
void key_scan_handler() //10ms
{
    // my_timer.stop();
    // printf("%lld   ,whther cording:%d    yaw:%f   current(%f,%f)   ,target(%f,%f)     \r",
    //     my_timer.elapsed_us(),
    //     (int)path_tracker_component.is_recording,
    //     ahrs.getYaw(),
    //     path_tracker_component.current_location.x,
    //     path_tracker_component.current_location.y,
    //     path_tracker_component.target_point.x,
    //     path_tracker_component.target_point.y
    // );
    // my_timer.start();
    key_manager.scan_keys();  // 执行按键扫描
    imu963r.update();         // 获取原始数据
   
    ahrs.updateIMU(
        imu963r.gyro[0]*IMU_calibration,  imu963r.gyro[2]*IMU_calibration, -imu963r.gyro[1]*IMU_calibration, 
        imu963r.acc[0],   imu963r.acc[2],  -imu963r.acc[1]   
    );
    // 路径记录或定位
    if(path_tracker_component.is_recording){
        path_tracker_component.record_sample(ahrs.getYaw());
    }else if (path_tracker_component.is_reproduction)
    {
        path_tracker_component.get_location(ahrs.getYaw());
    }
    
    
    //九轴陀螺仪综合测试，圆周运动
     // //需要进行对北操作
    // ahrs.update(
    //     imu963r.gyro[2], imu963r.gyro[0], -imu963r.gyro[1], // gx, gy, gz
    //     imu963r.acc[2],  imu963r.acc[0],  -imu963r.acc[1],  // ax, ay, az
    //     imu963r.mag[2],  imu963r.mag[0],  -imu963r.mag[1]   // mx, my, mz
    // );

    // //磁力轴抗压测试
    // float temp_sign = motor_test_signal_generator(1,1000);
    // speed_to_pwm_l = temp_sign;
    // speed_to_pwm_r = -temp_sign;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器采集线程回调函数
// 参数说明     无
// 返回参数     void
// 调用周期     ENCODER_SAMPLING_PERIOD (3ms)
// 线程优先级   99（最高优先级）
// 备注信息     ⚠️关键任务：读取编码器计数并映射为速度值，为PID控制提供实时反馈
//              此函数必须以最高优先级运行，确保速度采样的实时性和准确性
//-------------------------------------------------------------------------------------------------------------------
void encoder_get_count_handler()
{
    // -------------------- 周期测试 --------------------
    // my_timer.stop();                                         // 停止计时
    // printf("耗时: %lld us\n", my_timer.elapsed_us());        // 打印耗时
    // my_timer.start();                                        // 重新启动计时  
    // -------------------- 核心功能：获取编码器速度 --------------------
    // 读取编码器计数值并映射为速度，同时清零编码器计数器
    get_and_remap_speed(&right_speed, &left_speed, ENCODER_SAMPLING_PERIOD);
    // 开启路径记录时进行里程计更新
    if(path_tracker_component.is_recording){
        path_tracker_component.right_tyre.update(((int16_t)right_speed));
        path_tracker_component.left_tyre.update(((int16_t)left_speed));
    }else if (path_tracker_component.is_reproduction)
    {
        path_tracker_component.right_tyre.update(((int16_t)right_speed));
        path_tracker_component.left_tyre.update(((int16_t)left_speed));
    }else{
        // path_tracker_component.right_tyre.reset();
        // path_tracker_component.left_tyre.reset();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID速度控制线程回调函数
// 参数说明     无
// 返回参数     void
// 调用周期     PID_CONTROL_PERIOD (15ms)
// 线程优先级   97
// 备注信息     执行PID速度闭环控制，根据目标速度和当前速度计算PWM输出
//              处理流程：读取target_speed → PID计算 → 输出PWM → 驱动电机
//              ⚠️注意：默认情况下，此线程启动后立即开始控制，若需电机停止，请设置target_speed=0
//-------------------------------------------------------------------------------------------------------------------
void pid_contol_handle()
{
    // // 将PID计算结果发送到电机驱动模块
    motor_set_speed(speed_to_pwm_l, speed_to_pwm_r);

    // //测试代码
    if(onto_pd_control_enable==1){
        target_speed_l = target_speed_r = cruising_speed;//添加基准速度
        float onto_control = pid_angle.compute(0.0f, onto); // 计算角度修正值
        // float onto_control = pid_angle.compute(0.0f, -calculate_yaw_control(0,ahrs.getYaw(),40.0f)); // 计算角度修正值
        // float onto_control = calculate_yaw_control(0,ahrs.getYaw());
        // printf("onto: %f   ,onto_control: %f  \r",onto,onto_control);

        speed_to_pwm_r = (int16_t)pid_r.control(target_speed_r-onto_control, right_speed);  // 右轮PID计算
        speed_to_pwm_l = (int16_t)pid_l.control(target_speed_l+onto_control, left_speed);   // 左轮PID计算
    }
    else{
        // speed_to_pwm_r = (int16_t)pid_r.control(0, right_speed);  // 右轮PID计算
        // speed_to_pwm_l = (int16_t)pid_l.control(0, left_speed);   // 左轮PID计算
    }

    // // 惯性导航测试------------------------
    // if(path_tracker_component.is_reproduction){
    //     target_speed_l = target_speed_r = 10;//添加基准速度
    //     // 计算当前偏航角
    //     float onto_control = pid_angle.compute(0.0f, calculate_yaw_control(0,path_tracker_component.calculate_target_yaw(),20.0f)); // 计算角度修正值

    //     speed_to_pwm_r = (int16_t)pid_r.control(target_speed_r-onto_control, right_speed);  // 右轮PID计算
    //     speed_to_pwm_l = (int16_t)pid_l.control(target_speed_l+onto_control, left_speed);   // 左轮PID计算
    // }
    // else{
    //     speed_to_pwm_r = 0;
    //     speed_to_pwm_l = 0;
    // }
}

