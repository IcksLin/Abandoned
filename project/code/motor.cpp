/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-23        HeavenCornerstone         
* 2026-01-24        HeavenCornerstone         重新校验参数，进行了速度映射，此文件中的速度映射进行了实际采样和参数界定
*                                             如果月移植不根据当前车模实际情况重新界定参数，铁定不能用
*                                             后期控制修改若有周期偏移一定要重新界定参数
********************************************************************************************************************/


#include "motor.hpp"
//编码器组件
zf_driver_encoder encoder1(ZF_ENCODER_DIR_1);
zf_driver_encoder encoder2(ZF_ENCODER_DIR_2);

zf_driver_pwm right_motor(ZF_PWM_MOTOR_1);
zf_driver_pwm left_motor(ZF_PWM_MOTOR_2);
zf_driver_gpio right_dir(ZF_GPIO_MOTOR_1);
zf_driver_gpio left_dir(ZF_GPIO_MOTOR_2);

bool motor_init(){
    right_dir.set_level(1);
    left_dir.set_level(1);
    right_motor.set_duty(0);
    left_motor.set_duty(0);
    return true;
}

//============================================================================
// ============================ 宏定义区域 ============================
// 电机安全限幅（PWM值范围0~10000）
#define MOTOR_PWM_MAX        7200    // 最大PWM输出，留有余量保护电机
#define MOTOR_PWM_MIN        (-MOTOR_PWM_MAX)  // 负向最大

// 编码器到PWM的映射参数
#define ENCODER_MAX_PER_3MS  170     // 编码器3ms最大计数值（满速）
#define TARGET_SPEED_MAX     120     // 控制器最大目标速度

// PID输出到PWM的映射比例
#define PWM_PER_SPEED_UNIT   60      // 每个速度单位对应的PWM值
// 解释：PWM = 速度误差 × 这个系数，再叠加积分等

// 死区设置（防止零位振荡）
#define PWM_DEAD_ZONE        50      // PWM死区，小于此值不输出

// ============================ 函数实现 ============================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置电机速度（已优化映射和限幅）
// 参数说明 right_speed&left_speed PID控制器输出的速度指令，范围-120~120（负值表示反转）
//-------------------------------------------------------------------------------------------------------------------
void motor_set_speed(int16_t left_speed, int16_t right_speed)
{
    int16_t left_pwm, right_pwm;
    
    // 1. 映射PID速度指令到PWM值（核心映射）
    // 使用线性映射：PWM = 速度指令 × 比例系数
    left_pwm = left_speed * PWM_PER_SPEED_UNIT;
    right_pwm = right_speed * PWM_PER_SPEED_UNIT;
    
    // 2. 死区处理（消除零位抖动）
    if (abs(left_pwm) < PWM_DEAD_ZONE) {
        left_pwm = 0;
    }
    if (abs(right_pwm) < PWM_DEAD_ZONE) {
        right_pwm = 0;
    }
    
    // 3. 硬限幅保护电机
    // 正限幅
    if (left_pwm > MOTOR_PWM_MAX) {
        left_pwm = MOTOR_PWM_MAX;
    }
    if (right_pwm > MOTOR_PWM_MAX) {
        right_pwm = MOTOR_PWM_MAX;
    }
    // 负限幅
    if (left_pwm < MOTOR_PWM_MIN) {
        left_pwm = MOTOR_PWM_MIN;
    }
    if (right_pwm < MOTOR_PWM_MIN) {
        right_pwm = MOTOR_PWM_MIN;
    }
    
    // 4. 设置电机方向和PWM
    // ight_speed控制左电机，left_speed控制右电机，线插反了导致的
    if (right_pwm <= 0) {  // 控制左电机
        left_dir.set_level(1);      // 反转
        left_motor.set_duty(-right_pwm);
    } else {
        left_dir.set_level(0);      // 正转
        left_motor.set_duty(right_pwm);
    }
    
    if (left_pwm <= 0) {   // 控制右电机
        right_dir.set_level(1);     // 反转
        right_motor.set_duty(-left_pwm);
    } else {
        right_dir.set_level(0);     // 正转
        right_motor.set_duty(left_pwm);
    }
}

void motor_stop(){
    motor_set_speed(0,0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取电机速度
// 参数说明 right_speed&left_speed 速度值
// 参数说明 sampling_period 所启用的采样线程周期，单位ms，不建议超过1000,会触发逐飞int16上溢出,同时建议使用最高优先级
// 返回参数 bool
// 备注： 该函数在中断线程中调用
//-------------------------------------------------------------------------------------------------------------------
#define REMAP_WEIGHT 1.0f
bool get_and_remap_speed(float* right_speed,float*left_speed,int16 sampling_period){
    if (sampling_period>1000)
    {
        printf("An overflow exception may occur,please adjest your sampling period\n");
        return false;
    }else{
        *right_speed = -((float)encoder1.get_count()*REMAP_WEIGHT);
        *left_speed  = ((float)encoder2.get_count()*REMAP_WEIGHT);
        encoder1.clear_count();
        encoder2.clear_count();
        return true;
    }

}