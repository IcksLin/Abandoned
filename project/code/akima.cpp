#include "akima.hpp"
#include <cmath>
#include <algorithm>
#include <fstream>
#include <iomanip>

bool AkimaInterpolator::compute(const std::vector<double>& s, const std::vector<double>& v) {
    size_t n = s.size();
    if (n < 5) return false; // Akima 算法至少需要 5 个点来保证局部斜率估计的稳定性

    clear();
    s_knots = s;

    // 1. 计算相邻点间的斜率 m
    // 我们需要预留两端的虚拟斜率，所以大小为 n + 3
    std::vector<double> m(n + 3);
    for (size_t i = 0; i < n - 1; ++i) {
        m[i + 2] = (v[i + 1] - v[i]) / (s[i + 1] - s[i]);
    }

    // 2. 边界外推: 计算两端的虚拟斜率 (Virtual Slopes)
    // 这是 Akima 算法处理边界的经典方式
    m[1] = 2.0 * m[2] - m[3];
    m[0] = 2.0 * m[1] - m[2];
    m[n + 1] = 2.0 * m[n] - m[n - 1];
    m[n + 2] = 2.0 * m[n + 1] - m[n];

    // 3. 计算每个节点处的导数 t (即节点的切线斜率)
    std::vector<double> t(n);
    for (size_t i = 0; i < n; ++i) {
        // Akima 权重计算公式
        double w1 = std::abs(m[i + 3] - m[i + 2]);
        double w2 = std::abs(m[i + 1] - m[i]);
        
        if ((w1 + w2) > 1e-9) {
            // 标准加权平均
            t[i] = (w1 * m[i + 1] + w2 * m[i + 2]) / (w1 + w2);
        } else {
            // 如果分母接近0，说明局部是直线，取算术平均
            t[i] = 0.5 * (m[i + 1] + m[i + 2]);
        }
    }

    // 4. 计算每一段 [s_i, s_{i+1}] 上的立方多项式系数
    for (size_t i = 0; i < n - 1; ++i) {
        double h = s[i + 1] - s[i];
        AkimaCoeffs seg;
        seg.s0 = s[i];
        seg.d = v[i];                                          // f(s0) = v_i
        seg.c = t[i];                                          // f'(s0) = t_i
        seg.b = (3.0 * m[i + 2] - 2.0 * t[i] - t[i + 1]) / h;  // 二次项
        seg.a = (t[i] + t[i + 1] - 2.0 * m[i + 2]) / (h * h); // 三次项
        coeffs.push_back(seg);
    }

    return true;
}

double AkimaInterpolator::evaluate(double target_s) const {
    if (coeffs.empty()) return 0.0;

    // 二分查找确定 target_s 落在哪个区间
    auto it = std::upper_bound(s_knots.begin(), s_knots.end(), target_s);
    int idx = std::distance(s_knots.begin(), it) - 1;

    // 边界约束
    if (idx < 0) idx = 0;
    if (idx >= static_cast<int>(coeffs.size())) idx = coeffs.size() - 1;

    double ds = target_s - coeffs[idx].s0;
    
    // 使用秦九韶算法 (Horner's Method) 计算多项式值，减少乘法次数，提高嵌入式效率
    // y = d + ds * (c + ds * (b + ds * a))
    return coeffs[idx].d + ds * (coeffs[idx].c + ds * (coeffs[idx].b + ds * coeffs[idx].a));
}

void AkimaInterpolator::clear() {
    s_knots.clear();
    coeffs.clear();
}

void AkimaInterpolator::save_as_tracking_map(const AkimaInterpolator& x_interp, 
                                             const AkimaInterpolator& y_interp,
                                             double total_s, 
                                             double resolution, 
                                             const std::string& filename) {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) return;

    int index = 0;
    // 智能采样循环
    // 以 resolution 为步长从 0 走到 total_s
    for (double s = 0; s <= total_s; s += resolution) {
        double x = x_interp.evaluate(s);
        double y = y_interp.evaluate(s);

        // 写入格式：索引 x y
        outfile << index << " " 
                << std::fixed << std::setprecision(3) << x << " " 
                << std::fixed << std::setprecision(3) << y << "\n";
        
        index++;
    }

    // 强行补充终点，确保轨迹闭合且完整
    // 如果最后一个采样点距离终点超过 0.1 个单位，则补上终点
    if (std::fmod(total_s, resolution) > 0.001) {
        outfile << index << " " 
                << std::fixed << std::setprecision(3) << x_interp.evaluate(total_s) << " " 
                << std::fixed << std::setprecision(3) << y_interp.evaluate(total_s) << "\n";
    }

    outfile.close();
}