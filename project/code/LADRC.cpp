#include "LADRC.hpp"
#include <cmath>
#include <algorithm>

constexpr float LADRCPresets::DEFAULT_PARAMS[5][5];


LADRC::LADRC()
    : v1(0.0f), v2(0.0f),
      z1(0.0f), z2(0.0f), z3(0.0f),
      u(0.0f),
      h(0.001f), r(120.0f), wc(33.0f), w0(203.0f), b0(80.0f),
      max_output(1000.0f), min_output(-1000.0f) {
    // 使用默认参数初始化
}

LADRC::LADRC(float h_init, float r_init, float wc_init, float w0_init, float b0_init,float output_max,float output_min)
    : v1(0.0f), v2(0.0f),
      z1(0.0f), z2(0.0f), z3(0.0f),
      u(0.0f),
      h(h_init), r(r_init), wc(wc_init), w0(w0_init), b0(b0_init),
      max_output(output_max),min_output(output_min) {
    // 使用自定义参数初始化
}


bool LADRC::initWithPreset(unsigned int preset_index) {
    if (preset_index >= 5) {
        return false;  // 索引越界
    }
    
    h = LADRCPresets::DEFAULT_PARAMS[preset_index][0];
    r = LADRCPresets::DEFAULT_PARAMS[preset_index][1];
    wc = LADRCPresets::DEFAULT_PARAMS[preset_index][2];
    w0 = LADRCPresets::DEFAULT_PARAMS[preset_index][3];
    b0 = LADRCPresets::DEFAULT_PARAMS[preset_index][4];
    
    reset();
    return true;
}

void LADRC::initWithParameters(float h_new, float r_new, float wc_new, 
                               float w0_new, float b0_new) {
    h = h_new;
    r = r_new;
    wc = wc_new;
    w0 = w0_new;
    b0 = b0_new;
    
    reset();
}

void LADRC::reset() {
    v1 = 0.0f;
    v2 = 0.0f;
    z1 = 0.0f;
    z2 = 0.0f;
    z3 = 0.0f;
    u = 0.0f;
}

float LADRC::update(float expected_value, float measured_value) {
    // 第一步：跟踪微分 TD
    trackingDifferentiator(expected_value);
    
    // 第二步：扩展状态观测器 ESO
    extendedStateObserver(measured_value);
    
    // 第三步：线性反馈律
    linearFeedback();
    
    return u;
}

LADRCParameters LADRC::getParameters() const {
    return LADRCParameters{h, r, wc, w0, b0};
}

void LADRC::setBandwidth(float wc_new, float w0_new) {
    wc = wc_new;
    w0 = w0_new;
}

void LADRC::setTrackingSpeed(float r_new) {
    r = r_new;
}

void LADRC::setSamplingTime(float h_new) {
    if (h_new > 0.0f) {
        h = h_new;
    }
}

void LADRC::setSystemParameter(float b0_new) {
    if (b0_new != 0.0f) {
        b0 = b0_new;
    }
}

void LADRC::setOutputLimit(float max_val,float min_val){
    min_output = min_val;
    max_output = max_val;
}

void LADRC::getTrackerOutput(float& v1_out, float& v2_out) const {
    v1_out = v1;
    v2_out = v2;
}

void LADRC::getObserverOutput(float& z1_out, float& z2_out, float& z3_out) const {
    z1_out = z1;
    z2_out = z2;
    z3_out = z3;
}

void LADRC::trackingDifferentiator(float expected_value) {
    /**
     * @brief 跟踪微分算法
     * 
     * 动力学方程：
     *   dv1/dt = v2
     *   dv2/dt = -r²(v1 - v0) - 2r·v2
     * 
     * 其中：
     *   v0: 输入期望值
     *   v1: 位置跟踪输出
     *   v2: 速度微分输出
     *   r: 跟踪速度因子（越大跟踪越快）
     */
    
    float fh = -r * r * (v1 - expected_value) - 2.0f * r * v2;
    
    // 欧拉离散化
    v1 += v2 * h;
    v2 += fh * h;
}

// void LADRC::extendedStateObserver(float feedback_value) {//(二阶)
//     /**
//      * @brief 线性扩展状态观测器 (Linear ESO)
//      * 
//      * 状态方程：
//      *   dz1/dt = z2 - β₁·e
//      *   dz2/dt = z3 - β₂·e + b₀·u
//      *   dz3/dt = -β₃·e
//      * 
//      * 其中：
//      *   z1: 观测的位置状态
//      *   z2: 观测的速度状态
//      *   z3: 观测的扰动/不确定项
//      *   e = z1 - y: 观测误差
//      *   y: 实际反馈值
//      *   β₁, β₂, β₃: 观测器带宽相关的参数
//      */
    
//     // 计算观测器增益参数
//     float beta_01 = 3.0f * w0;
//     float beta_02 = 3.0f * w0 * w0;
//     float beta_03 = w0 * w0 * w0;
    
//     // 计算观测误差
//     float e = z1 - feedback_value;
    
//     // 离散化状态更新（欧拉法）
//     z1 += (z2 - beta_01 * e) * h;
//     z2 += (z3 - beta_02 * e + b0 * u) * h;
//     z3 += (-beta_03 * e) * h;
// }

void LADRC::extendedStateObserver(float feedback_value) {
    /**
     * @brief 一阶线性扩展状态观测器 (1st Order LESO)
     * 针对速度环：
     * dz1/dt = z2 - β₁·e + b₀·u  (z1 跟踪实际速度)
     * dz2/dt = -β₂·e             (z2 跟踪总扰动)
     */
    
    // 1. 计算一阶观测器增益 (根据带宽 w0)
    float beta_01 = 2.0f * w0;          // 一阶对应 2*w0
    float beta_02 = w0 * w0;            // 一阶对应 w0^2
    
    // 2. 计算观测误差
    float e = z1 - feedback_value;
    
    // 3. 状态更新 (欧拉离散化)
    // 注意：在一阶模型中，控制量 b0*u 直接作用在 z1 的变化上
    z1 += (z2 - beta_01 * e + b0 * u) * h;
    z2 += (-beta_02 * e) * h;
    
    // z3 不再使用，可以置零或忽略
    z3 = 0.0f; 
}

// void LADRC::linearFeedback() {//(二阶)
//     /**
//      * @brief 线性状态误差反馈 (Linear SEFE)
//      * 
//      * 控制律：
//      *   e₁ = v1 - z1  (位置偏差)
//      *   e₂ = v2 - z2  (速度偏差)
//      *   u₀ = Kp·e₁ + Kd·e₂  (PD控制)
//      *   u = (u₀ - z₃) / b₀  (抗扰动补偿)
//      * 
//      * 其中：
//      *   Kp = wc²: 比例增益
//      *   Kd = 2·wc: 微分增益
//      *   z₃: 扰动估计值
//      */
    
//     float Kp = wc * wc;
//     float Kd = 2.0f * wc;
    
//     // 计算位置和速度偏差
//     float e1 = v1 - z1;
//     float e2 = v2 - z2;
    
//     // PD控制律加扰动补偿
//     float u0 = Kp * e1 + Kd * e2;
    
//     // 补偿观测到的扰动
//     u = (u0 - z3) / b0;
    
//     // 饱和处理
//     u = saturate(u);
// }

void LADRC::linearFeedback() {
    /**
     * @brief 一阶线性状态误差反馈
     * 控制律：
     * u0 = Kp * (v1 - z1)
     * u = (u0 - z2) / b0
     */
    
    // 1. 计算比例增益 (一阶对应的 Kp 就是控制器带宽 wc)
    float Kp = wc; 
    
    // 2. 计算速度偏差 (期望速度 v1 - 观测速度 z1)
    float e1 = v1 - z1;
    
    // 3. 线性控制律
    float u0 = Kp * e1;
    
    // 4. 扰动补偿 (减去观测到的扰动 z2)
    u = (u0 - z2) / b0;
    
    // 5. 饱和处理
    u = saturate(u);
}

float LADRC::saturate(float value) const {
    if (value > max_output) {
        return max_output;
    } else if (value < min_output) {
        return min_output;
    }
    return value;
}