#ifndef __TFLM_MODEL_PROCESS_LQ_HPP__
#define __TFLM_MODEL_PROCESS_LQ_HPP__

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <net.h> // ncnn 头文件

struct LQ_InferenceResult {
    int class_index;
    std::string label;
    float confidence;
};

class LQ_NCNN {
public:
    LQ_NCNN();
    ~LQ_NCNN();

    // 初始化：加载 param 和 bin 文件
    bool init(const std::string& param_path, const std::string& bin_path,int input_width, int input_height);

    // 执行推理
    LQ_InferenceResult infer(const cv::Mat& bgr_image) const;

private:
    // 静态工具函数：寻找最大值索引
    static int argmax(const ncnn::Mat& logits, float& prob);

private:
    ncnn::Net net_;
    std::vector<std::string> labels_;
    bool initialized_ = false;

    // 模型输入尺寸（需与训练时保持一致）
    int kInputWidth = 40;
    int kInputHeight = 40;

    // 归一化参数 (ImageNet 标准)
    // ncnn 的 substract_mean_normalize 公式: (x - mean) * norm
    const float kMeanVals[3] = {123.675f, 116.28f, 103.53f};
    const float kNormVals[3] = {0.01712475f, 0.017507f, 0.01742919f};
};

#endif // __TFLM_MODEL_PROCESS_LQ_HPP__