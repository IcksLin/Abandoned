#ifndef AKIMA_INTERPOLATOR_H
#define AKIMA_INTERPOLATOR_H

#include <vector>
#include <string>
/**
 * @brief Akima 分段立方多项式系数
 * 表达式形式: f(s) = a(s-s0)^3 + b(s-s0)^2 + c(s-s0) + d
 */
struct AkimaCoeffs {
    double s0; // 区间起始自变量值（如起始里程）
    double a;  // 三次项系数
    double b;  // 二次项系数
    double c;  // 一次项系数
    double d;  // 常数项系数
};

class AkimaInterpolator {
public:
    AkimaInterpolator() = default;

    /**
     * @brief 计算 Akima 插值系数
     * @param s 自变量序列（必须单调递增，如累计里程）
     * @param v 因变量序列（对应的坐标 X 或 Y 或 Yaw）
     * @return true 成功, false 失败（点数少于5个）
     */
    bool compute(const std::vector<double>& s, const std::vector<double>& v);

    /**
     * @brief 根据输入值评估插值结果
     * @param target_s 目标自变量值
     * @return 插值后的因变量值
     */
    double evaluate(double target_s) const;

    /**
     * @brief 集成化地图保存功能
     * @param x_interp 已计算好系数的 X 轴插值器
     * @param y_interp 已计算好系数的 Y 轴插值器
     * @param total_s 路径总里程
     * @param resolution 等距采样的分辨率（单位：脉冲/mm）
     * @param filename 保存的文件名
     */
    static void save_as_tracking_map(const AkimaInterpolator& x_interp, 
                                     const AkimaInterpolator& y_interp,
                                     double total_s, 
                                     double resolution, 
                                     const std::string& filename);

    /**
     * @brief 清除当前插值器数据
     */
    void clear();

private:
    std::vector<double> s_knots;      // 存储原始自变量节点
    std::vector<AkimaCoeffs> coeffs;  // 存储计算好的各段系数矩阵
};

#endif