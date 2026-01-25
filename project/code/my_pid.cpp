/*********************************************************************************************************************
* 文件名称          my_pid
* 适用平台          LS2K0300
* 修改记录
* 日期              作者                        备注
* 2026-01-23        HeavenCornerstone         
* 2026-01-23        HeavenCornerstone          修正PID计算步骤
********************************************************************************************************************/



#include "my_pid.hpp"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 无
// 返回参数 无
// 使用示例 MyPID pid_controller;
// 备注信息 创建PID控制器对象，需要调用init()进行初始化
//-------------------------------------------------------------------------------------------------------------------
MyPID::MyPID()
{
    // 初始化所有参数为默认值
    Kp = 0.0f;
    Ki = 0.0f;
    Kd = 0.0f;
    Ti = 0.01f;
    integral = 0.0f;
    prev_error = 0.0f;
    last_error = 0.0f;
    before_last_error = 0.0f;
    error_filter = 0.0f;
    delta_error = 0.0f;
    output_max = 100.0f;
    output_min = -100.0f;
    output = 0.0f;
    integral_max = 1000.0f;
    integral_min = -1000.0f;
    
    // 扩展参数初始化
    derivative_filter = 1.0f; // 默认无滤波
    derivative_last = 0.0f;
    feedforward = 0.0f;
    enable_feedforward = false;
    reverse_action = false;
    enable = true;
    anti_windup = false;
    windup_zone = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 析构函数
// 参数说明 无
// 返回参数 无
// 使用示例 自动调用
// 备注信息 清理资源
//-------------------------------------------------------------------------------------------------------------------
MyPID::~MyPID()
{
    // 析构函数，清理资源
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 PID控制器初始化
// 参数说明 Kp              比例增益
// 参数说明 Ti              积分时间常数
// 参数说明 Kd              微分增益
// 参数说明 error_filter    死区阈值
// 参数说明 output_max      输出最大值
// 参数说明 output_min      输出最小值
// 参数说明 integral_max    积分最大值
// 参数说明 integral_min    积分最小值
// 返回参数 无
// 使用示例 pid.init(1.0f, 0.01f, 0.1f, 0.1f, 100.0f, -100.0f, 1000.0f, -1000.0f);
// 备注信息 初始化PID控制器的基本参数
//-------------------------------------------------------------------------------------------------------------------
void MyPID::init(float Kp, float Ti, float Kd,
                 float error_filter,
                 float output_max, float output_min,
                 float integral_max, float integral_min)
{
    this->Kp = Kp;
    this->Ti = Ti;
    this->Ki = Kp * Ti * 5.20f; // 积分增益，可根据实际调整
    this->Kd = Kd;
    this->integral = 0.0f;
    this->prev_error = 0.0f;
    this->last_error = 0.0f;
    this->before_last_error = 0.0f;
    this->error_filter = error_filter;
    this->delta_error = 0.0f;
    this->output_max = output_max;
    this->output_min = output_min;
    this->output = 0.0f;
    this->integral_max = integral_max;
    this->integral_min = integral_min;

    // 扩展参数保持默认值或之前设置的值
    this->derivative_filter = 1.0f; // 默认无滤波
    this->derivative_last = 0.0f;
    this->feedforward = 0.0f;
    this->enable_feedforward = false;
    this->reverse_action = false;
    this->enable = true;
    this->anti_windup = false;
    this->windup_zone = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 PID控制计算
// 参数说明 target          目标值
// 参数说明 current_value   当前值
// 返回参数 float           控制输出值
// 使用示例 float output = pid.control(100.0f, 95.0f);
// 备注信息 执行PID控制算法，返回控制输出
//-------------------------------------------------------------------------------------------------------------------
float MyPID::control(float target, float current_value)
{
    if (!enable)
        return 0.0f;

    float new_error;
    new_error = target - current_value;
    if (reverse_action)
        new_error = -new_error;

    before_last_error = last_error;
    last_error = prev_error;
    prev_error = new_error;
    delta_error = prev_error - last_error;

    // 死区处理
    if (PID_ABS(prev_error) <= error_filter)
    {
        prev_error = 0.0f;
    }

    // 积分分离/抗积分饱和 - 修复积分贡献限制逻辑
    float integral_contribution = 0.0f;
    if (anti_windup && (PID_ABS(prev_error) > windup_zone))
    {
        // 误差大于分离死区时不积分
    }
    else
    {
        // 计算本次的积分增量
        float integral_increment = prev_error;
        
        // 先计算如果加上增量后，积分贡献是否会超限
        float potential_integral = integral + integral_increment;
        float potential_contribution = Ki * potential_integral;
        
        // 检查积分贡献是否会超出允许范围
        // integral_max/min 应该表示 Ki * integral 的最大最小值
        if (potential_contribution > integral_max)
        {
            // 如果会超出上限，只累加到刚好达到上限
            integral = integral_max / Ki;
        }
        else if (potential_contribution < integral_min)
        {
            // 如果会超出下限，只累加到刚好达到下限
            integral = integral_min / Ki;
        }
        else
        {
            // 在范围内，正常累加
            integral = potential_integral;
        }
        
        // 计算最终的积分贡献
        integral_contribution = Ki * integral;
    }

    // 微分项低通滤波（抑制高频噪声）
    float derivative_raw = prev_error + before_last_error - 2 * last_error;
    float derivative = derivative_filter * derivative_raw + (1.0f - derivative_filter) * derivative_last;
    derivative_last = derivative;
    
    // 微分贡献
    float derivative_contribution = Kd * derivative;

    // 增量式PID公式 - 使用计算好的各项贡献
    output = Kp * prev_error + integral_contribution + derivative_contribution;

    // 前馈项
    if (enable_feedforward)
        output += feedforward;

    output = PID_LIMIT(output, output_min, output_max);

    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 重置PID控制器
// 参数说明 无
// 返回参数 无
// 使用示例 pid.reset();
// 备注信息 清零积分项和历史误差，重置控制器状态
//-------------------------------------------------------------------------------------------------------------------
void MyPID::reset(void)
{
    integral = 0.0f;
    prev_error = 0.0f;
    last_error = 0.0f;
    before_last_error = 0.0f;
    delta_error = 0.0f;
    output = 0.0f;
    derivative_last = 0.0f;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置积分时间常数
// 参数说明 Ti              积分时间常数
// 返回参数 无
// 使用示例 pid.set_ti(0.02f);
// 备注信息 动态调整积分时间常数，会自动重新计算Ki
//-------------------------------------------------------------------------------------------------------------------
void MyPID::set_ti(float Ti)
{
    this->Ti = PID_LIMIT(Ti, 0.01f, 100.0f); // 防止Ti为0
    this->Ki = this->Kp / this->Ti;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置PID参数
// 参数说明 kp              比例增益
// 参数说明 kd              微分增益
// 参数说明 ki_change       积分增益
// 参数说明 dt              采样时间（保留兼容性，暂未使用）
// 返回参数 无
// 使用示例 pid.set_index(1.5f, 0.1f, 0.05f, 0.01f);
// 备注信息 直接设置PID三个参数
//-------------------------------------------------------------------------------------------------------------------
void MyPID::set_index(float kp, float kd, float ki_change, float dt)
{
    this->Kp = kp;
    this->Kd = kd;
    this->Ki = ki_change;
    // dt参数保留兼容性，暂未使用
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置PID使能状态
// 参数说明 enable          使能状态
// 返回参数 无
// 使用示例 pid.set_enable(true);
// 备注信息 禁用时会清零积分项和输出
//-------------------------------------------------------------------------------------------------------------------
void MyPID::set_enable(bool enable)
{
    this->enable = enable;
    if (!enable)
    {
        integral = 0.0f;
        output = 0.0f;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置反向控制
// 参数说明 reverse_action  反向控制开关
// 返回参数 无
// 使用示例 pid.set_reverse(true);
// 备注信息 反向控制时误差符号会反转
//-------------------------------------------------------------------------------------------------------------------
void MyPID::set_reverse(bool reverse_action)
{
    this->reverse_action = reverse_action;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置前馈控制
// 参数说明 ff              前馈值
// 参数说明 enable_ff       前馈使能
// 返回参数 无
// 使用示例 pid.set_feedforward(50.0f, true);
// 备注信息 前馈值会直接加到PID输出上
//-------------------------------------------------------------------------------------------------------------------
void MyPID::set_feedforward(float ff, bool enable_ff)
{
    this->feedforward = ff;
    this->enable_feedforward = enable_ff;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置微分低通滤波
// 参数说明 filter_coeff    滤波系数（0~1，0为最大平滑）
// 返回参数 无
// 使用示例 pid.set_derivative_filter(0.7f);
// 备注信息 用于抑制微分项的高频噪声
//-------------------------------------------------------------------------------------------------------------------
void MyPID::set_derivative_filter(float filter_coeff)
{
    if (filter_coeff < 0.0f)
        filter_coeff = 0.0f;
    if (filter_coeff > 1.0f)
        filter_coeff = 1.0f;
    this->derivative_filter = filter_coeff;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置积分分离
// 参数说明 enable          积分分离使能
// 参数说明 windup_zone     积分分离死区
// 返回参数 无
// 使用示例 pid.set_anti_windup(true, 10.0f);
// 备注信息 误差大于死区时不进行积分，防止积分饱和
//-------------------------------------------------------------------------------------------------------------------
void MyPID::set_anti_windup(bool enable, float windup_zone)
{
    this->anti_windup = enable;
    this->windup_zone = windup_zone;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取当前输出值
// 参数说明 无
// 返回参数 float           当前输出值
// 使用示例 float current_output = pid.get_output();
// 备注信息 获取最近一次控制计算的输出值
//-------------------------------------------------------------------------------------------------------------------
float MyPID::get_output(void) const
{
    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取当前误差
// 参数说明 无
// 返回参数 float           当前误差值
// 使用示例 float error = pid.get_error();
// 备注信息 获取最近一次的误差值
//-------------------------------------------------------------------------------------------------------------------
float MyPID::get_error(void) const
{
    return prev_error;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取积分项
// 参数说明 无
// 返回参数 float           当前积分值
// 使用示例 float integral_val = pid.get_integral();
// 备注信息 获取当前积分累积值
//-------------------------------------------------------------------------------------------------------------------
float MyPID::get_integral(void) const
{
    return integral;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取使能状态
// 参数说明 无
// 返回参数 bool            使能状态
// 使用示例 bool is_enabled = pid.is_enabled();
// 备注信息 检查PID控制器是否处于使能状态
//-------------------------------------------------------------------------------------------------------------------
bool MyPID::is_enabled(void) const
{
    return enable;
}