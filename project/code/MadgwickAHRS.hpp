//=============================================================================================
// MadgwickAHRS.hpp
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// C++ version with object-oriented design
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 2026/02/04   C++ Port        Object-oriented C++ implementation
//
//=============================================================================================

#ifndef MADGWICK_AHRS_HPP
#define MADGWICK_AHRS_HPP

#include <cmath>
#include <cstdint>
#include <cstring>

class MadgwickAHRS {
public:
    /**
     * @brief 构造姿态解算器
     * @param sampleFrequency 解算调用频率，单位 Hz
     * @param beta 算法收敛增益，越大越依赖加速度/磁力计修正
     */
    MadgwickAHRS(float sampleFrequency = 100.0f, float beta = 0.1f);
    
    // Destructor
    ~MadgwickAHRS() = default;
    
    // Copy constructor and assignment operator
    MadgwickAHRS(const MadgwickAHRS& other) = default;
    MadgwickAHRS& operator=(const MadgwickAHRS& other) = default;
    
    // Move constructor and assignment operator
    MadgwickAHRS(MadgwickAHRS&& other) noexcept = default;
    MadgwickAHRS& operator=(MadgwickAHRS&& other) noexcept = default;
    
    /**
     * @brief 设置解算频率
     * @param frequency 调用频率，单位 Hz，必须大于 0
     */
    void setSampleFrequency(float frequency);

    /**
     * @brief 设置 Madgwick 算法增益
     * @param beta 收敛增益，必须大于 0
     */
    void setBeta(float beta);
    float getSampleFrequency() const { return 1.0f / invSampleFreq_; }
    float getBeta() const { return beta_; }
    
    /**
     * @brief 九轴 AHRS 更新
     * @param gx X 轴角速度，单位 deg/s
     * @param gy Y 轴角速度，单位 deg/s
     * @param gz Z 轴角速度，单位 deg/s
     * @param ax X 轴加速度
     * @param ay Y 轴加速度
     * @param az Z 轴加速度
     * @param mx X 轴磁力计
     * @param my Y 轴磁力计
     * @param mz Z 轴磁力计
     */
    void update(float gx, float gy, float gz, 
                float ax, float ay, float az, 
                float mx, float my, float mz);

    /**
     * @brief 六轴 IMU 更新
     * @param gx X 轴角速度，单位 deg/s
     * @param gy Y 轴角速度，单位 deg/s
     * @param gz Z 轴角速度，单位 deg/s
     * @param ax X 轴加速度
     * @param ay Y 轴加速度
     * @param az Z 轴加速度
     * @note 当前工程主要调用该函数，传参顺序和符号需要按 IMU 实际安装方向调整。
     */
    void updateIMU(float gx, float gy, float gz, 
                   float ax, float ay, float az);
    
    // Angle getters (degrees)
    float getRoll();
    float getPitch();
    float getYaw();
    
    // Angle getters (radians)
    float getRollRadians();
    float getPitchRadians();
    float getYawRadians();
    
    // Quaternion getters
    void getQuaternion(float& q0, float& q1, float& q2, float& q3) const;
    
    // Reset filter
    void reset();
    
    // Utility methods
    bool isAnglesComputed() const { return anglesComputed_; }
    
private:
    // Algorithm parameters
    float beta_;                    // Algorithm gain
    float invSampleFreq_;          // Inverse sample frequency
    
    // Quaternion components
    float q0_, q1_, q2_, q3_;      // Quaternion of sensor frame relative to auxiliary frame
    
    // Euler angles (radians)
    float roll_, pitch_, yaw_;
    bool anglesComputed_;
    
    // Private methods
    void computeAngles();
    static float invSqrt(float x);
    
    // Constants
    static constexpr float DEFAULT_SAMPLE_FREQ = 100.0f;
    static constexpr float DEFAULT_BETA = 0.1f;
    static constexpr float RAD_TO_DEG = 57.29578f;
};

#endif // MADGWICK_AHRS_HPP
