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
float cruising_speed = CRUISING_SPEED;                          // 巡航速度，修改该值会直接影响小车速度
float onto_control = 0;                                         // 方向PD控制量，跨线程传递参数

// -------------------- PWM输出变量（系统内部使用） --------------------
int16_t speed_to_pwm_r = 0;                                     // 右轮PWM输出值（PID计算结果）
int16_t speed_to_pwm_l = 0;                                     // 左轮PWM输出值（PID计算结果）

// // -------------------- PID控制器对象 --------------------
MyPID pid_r;                                                    // 右轮PID控制器
MyPID pid_l;                                                    // 左轮PID控制器
PDController pid_angle;                                         // 角度PID控制器
// ladrc控制方案--------------------------------------------------
SimpleMotorLADRC ladrc_left;
SimpleMotorLADRC ladrc_right;
SimpleMotorLADRC ladrc_onto_control;

// 初始化线程，可开始参数获取任务调度
zf_driver_pit_rt encoder_get;
zf_driver_pit_rt pid_control_thread;
zf_driver_pit_rt key_scan;
zf_driver_pit_rt lardc_control_thread;


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
    key_manager.scan_keys();  // 执行按键扫描
    imu963r.update();         // 获取原始数据
   
    ahrs.updateIMU(
        imu963r.gyro[0]*IMU_calibration,  imu963r.gyro[2]*IMU_calibration, -imu963r.gyro[1]*IMU_calibration, 
        imu963r.acc[0],   imu963r.acc[2],  -imu963r.acc[1]   
    );
    // // 路径记录或定位
    // if(path_tracker_component.is_recording){
    //     path_tracker_component.record_sample(ahrs.getYaw());
    // }else if (path_tracker_component.is_reproduction)
    // {
    //     path_tracker_component.get_location(ahrs.getYaw());
    // }
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

void hight_frequence_encoder_get_speed_handler(){
    lardc_get_speed(1,1024,0.035,0.001, &left_speed,&right_speed);
    if(onto_pd_control_enable==1){
        target_speed_l = target_speed_r = cruising_speed;//添加基准速度

        // speed_to_pwm_l = (int16_t)ladrc_left.calculatePWM(0, left_speed);
        // speed_to_pwm_r = (int16_t)ladrc_right.calculatePWM(-0, right_speed);

        speed_to_pwm_l = (int16_t)ladrc_left.calculatePWM(target_speed_l+onto_control, left_speed);
        speed_to_pwm_r = (int16_t)ladrc_right.calculatePWM(target_speed_r-onto_control, right_speed);
    }
    else{
        // speed_to_pwm_l = 0;
        // speed_to_pwm_r = 0;
    }
    motor_set_speed_ladrc(speed_to_pwm_l, speed_to_pwm_r);
    // motor_set_speed_ladrc(300, 300);

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
    static float ti_tick = 0;
    //测试代码
    if(onto_pd_control_enable==1){
        // onto_control = pid_angle.compute(onto, 0.0f); // 计算角度修正值
        // printf("onto : %f  ,ontoControl:  %f   ,time : %lld  \r",onto,onto_control,my_timer.elapsed_ms());
        onto_control = pid_angle.compute(calculate_yaw_control(90.0f * (sin(ti_tick / 20.0f) >= 0 ? 1.0f : -1.0f),ahrs.getYaw(), 25),0); // 计算角度修正值
    }
    else{
        onto_control = 0;
    }
    // calculate_yaw_control
    // onto_control = pid_angle.compute(onto, 0.0f); // 计算角度修正值
    // printf("onto : %f  ,ontoControl:  %f   ,time : %lld  \r",onto,onto_control,my_timer.elapsed_ms());
}

bool car_init(){
    // 导入透视变换矩阵
    save_per_map();
    // 导入基础参数并设置
    param_loading_from_file("/home/root/car_config.txt");
    param_print();
    // 1. 初始化显示屏
    printf("------------初始化显示屏-----------\n");
    ips200.init("/dev/fb0");
    ips200.clear();
    ips200.show_string(10, 10, "Camera & Encoder");
    ips200.show_string(10, 30, "System Ready");
    ips200.update();
    system_delay_ms(100);

    // 2. 初始化编码器（清零采样值）
    printf("------------初始化编码器------------\n");
    encoder1.clear_count();
    encoder2.clear_count();
    printf("编码器已清零\n");
    system_delay_ms(100);

    // 3.初始化DRV驱动
    printf("------------初始化DRV驱动------------\n");
    motor_init();
    left_speed = 0;
    right_speed = 0;
    motor_set_speed(left_speed, right_speed);

    // 4. 初始化摄像头
    printf("------------初始化摄像头------------\n");
    uvc.init("/dev/video0");

    // 5.初始化陀螺仪
    printf("------------初始化陀螺仪------------\n");
    imu963r.init();
    system_delay_ms(100);

    // 6. 初始化菜单系统
    printf("------------初始化菜单系统------------\n");
    // menu_system.init_menu();

    // printf("8. 初始化图像分类组件...\n");
    // if (!ncnn_classifier.init("/home/root/models/model_1/tiny_classifier_fp32.ncnn.param", "/home/root/models/model_1/tiny_classifier_fp32.ncnn.bin")) {
    //     printf("NCNN模型初始化失败！\n");
    //     return false;
    // }

    // TCP图像传输组件初始化 --- IGNORE ---
    // printf("9. TCP图像传输组件初始化...\n");
    // if (udp.init("192.168.43.94",8086)) {
    //     printf("NCNN模型初始化失败！\n");
    //     return false;
    // }

    // printf("9. TCP图像传输组件初始化...\n");
    // img_transmitter_init();

    if(control_model == 0){
        printf("pid control model");
        if (pid_control_thread.init_ms(PID_CONTROL_PERIOD, pid_contol_handle, 97, true) != 0)
        {
            printf("PID控制器线程初始化失败");
            return false;
        }
        else
        {
            printf("pid control thread init successfully,period:%dms\n", PID_CONTROL_PERIOD);
        }

        if (encoder_get.init_ms(ENCODER_SAMPLING_PERIOD, encoder_get_count_handler, 99, true) != 0)
        {
            printf("编码器获取线程初始化失败\n");
            return false;
        }
        else
        {
            printf("encoder geting count thread init successfully,period: %dms\n", ENCODER_SAMPLING_PERIOD);
        }

    }else{
        if (lardc_control_thread.init_ms(LARDC_PERIOD, hight_frequence_encoder_get_speed_handler, 99, true) != 0)
        {
            printf("lardc控制器线程初始化失败");
            return false;
        }
        else
        {
            printf("lardc control thread init successfully,period:%dms\n", LARDC_PERIOD);
        }

        // 方向PD
        if (pid_control_thread.init_ms(PID_CONTROL_PERIOD, pid_contol_handle, 97, true) != 0)
        {
            printf("PID控制器线程初始化失败");
            return false;
        }
        else
        {
            printf("pid control thread init successfully,period:%dms\n", PID_CONTROL_PERIOD);
        }
    }
    
    if (key_scan.init_ms(KEY_SCAN_PERIOD, key_scan_handler, 95, true) != 0)
    {
        printf("定时器初始化失败\n");
        return false;
    }
    else
    {
        printf("key scaning thread init successfully,period: %dms\n", KEY_SCAN_PERIOD);
    }
    return true;
}