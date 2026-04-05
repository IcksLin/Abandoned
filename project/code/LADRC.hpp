#ifndef _LADRC_HPP_
#define _LADRC_HPP_

#include <cmath>
#include <algorithm>
/**
 * @brief LADRC参数结构体
 */
struct LADRCParameters {
    float h;    ///< 积分步长 (sampling time)
    float r;    ///< 跟踪速度因子 (tracking speed)
    float wc;   ///< 控制器带宽
    float w0;   ///< 观测器带宽
    float b0;   ///< 系统参数
};

/**
 * @brief LADRC参数自整定配置
 */
struct LADRCAutoTuning {
    float Pu;   ///< 继电实验输出周期
    float a;    ///< 继电实验输出幅值
    float h;    ///< 指令输出幅值
    float Wu;   ///< 系统临界频率
    float Kp;   ///< 系统临界幅值
};

/**
 * @brief LADRC标准参数表配置集合
 * @details 包含5组预设参数，用于不同的应用场景
 */
struct LADRCPresets {
    static constexpr float DEFAULT_PARAMS[5][5] = {
        {0.005f, 20.0f, 100.0f, 400.0f, 0.5f},    // 预设 0
        {0.001f, 20.0f, 33.0f, 133.0f, 8.0f},     // 预设 1 (默认)
        {0.005f, 100.0f, 20.0f, 80.0f, 0.5f},     // 预设 2
        {0.005f, 100.0f, 14.0f, 57.0f, 0.5f},     // 预设 3
        {0.005f, 100.0f, 50.0f, 10.0f, 1.0f}      // 预设 4
    };
};

class LADRC {
public:
    LADRC();   
    /**
     * @brief 构造函数 - 使用指定参数初始化
     * @param h 积分步长 (sampling time)
     * @param r 跟踪速度因子
     * @param wc 控制器带宽
     * @param w0 观测器带宽
     * @param b0 系统参数
     * @param max 最大输出值
     * @param min 最小输出值
     */
    LADRC(float h, float r, float wc, float w0, float b0,float max,float min);

    ~LADRC() = default;
    
    
    /**
     * @brief 使用预设参数初始化 LADRC
     * @param preset_index 预设索引 [0-4]
     * @return true 初始化成功，false 索引无效
     */
    bool initWithPreset(unsigned int preset_index);
    
    /**
     * @brief 使用自定义参数初始化 LADRC
     * @param h 积分步长
     * @param r 跟踪速度因子
     * @param wc 控制器带宽
     * @param w0 观测器带宽
     * @param b0 系统参数
     */
    void initWithParameters(float h, float r, float wc, float w0, float b0);
    
    /**
     * @brief 重置 LADRC 状态变量
     * @details 将观测器输出 z1, z2, z3 重置为零，清空历史状态
     */
    void reset();
    
    /**
     * @brief 执行一次完整的 LADRC 控制循环
     * @param expected_value 期望值/参考值
     * @param measured_value 实际反馈值
     * @return 计算得到的控制输出 u
     * 
     * @details 此函数依次执行：
     *          1. 跟踪微分 (Tracking Differentiator)
     *          2. 线性扩展状态观测器 (ESO)
     *          3. 线性反馈律 (LSEF)
     */
    float update(float expected_value, float measured_value);
    
    /**
     * @brief 获取当前控制输出
     * @return 最新计算的控制输出值
     */
    float getOutput() const { return u; }

    /**
     * @brief 获取当前参数配置
     * @return LADRCParameters 结构体包含所有参数
     */
    LADRCParameters getParameters() const;
    
    /**
     * @brief 更新带宽参数
     * @param wc_new 新的控制器带宽
     * @param w0_new 新的观测器带宽
     */
    void setBandwidth(float wc_new, float w0_new);
    
    /**
     * @brief 更新跟踪微分参数
     * @param r_new 新的跟踪速度因子
     */
    void setTrackingSpeed(float r_new);
    
    /**
     * @brief 更新采样时间
     * @param h_new 新的积分步长
     */
    void setSamplingTime(float h_new);
    
    /**
     * @brief 更新系统参数
     * @param b0_new 新的系统参数 b0
     */
    void setSystemParameter(float b0_new);

    void setOutputLimit(float max_val,float min_val);
    
    /**
     * @brief 获取跟踪微分器输出
     * @param v1 位置输出 (reference: v1)
     * @param v2 速度输出 (reference: v2)
     */
    void getTrackerOutput(float& v1, float& v2) const;
    
    /**
     * @brief 获取扩展状态观测器输出
     * @param z1 观测位置
     * @param z2 观测速度
     * @param z3 观测扰动/不确定度
     */
    void getObserverOutput(float& z1, float& z2, float& z3) const;


private:

    
    // 跟踪微分器状态
    float v1, v2;
    
    // 扩展状态观测器状态
    float z1, z2, z3;
    
    // 控制输出
    float u;
    
    float h;    ///< 积分步长 (sampling time)
    float r;    ///< 跟踪速度因子 (tracking speed)
    float wc;   ///< 控制器带宽
    float w0;   ///< 观测器带宽
    float b0;   ///< 系统参数
    
    // 控制约束
    float max_output;
    float min_output;

    
    /**
     * @brief 跟踪微分 (Tracking Differentiator) TD
     * @details 对期望值进行平滑的跟踪和微分处理
     * @param expected_value 输入期望值
     */
    void trackingDifferentiator(float expected_value);
    
    /**
     * @brief 线性扩展状态观测器 (Linear Extended State Observer) ESO
     * @details 观测系统的状态和未知扰动
     * @param feedback_value 反馈测量值
     */
    void extendedStateObserver(float feedback_value);
    
    /**
     * @brief 线性反馈律 (Linear State Error Feedback) SEF
     * @details 计算控制律，包括饱和处理
     */
    void linearFeedback();
    
    /**
     * @brief 对输出值进行饱和处理
     * @param value 输入值
     * @return 饱和后的值
     */
    float saturate(float value) const;
};

#endif // _LADRC_HPP_