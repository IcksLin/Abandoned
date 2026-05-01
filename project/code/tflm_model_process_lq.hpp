/**
 * @file tflm_model_process_lq.hpp
 * @brief 基于 ncnn 的龙邱模型推理封装头文件
 * @note 文件名保留了早期 TFLM 版本命名，当前实际使用的是 ncnn 推理后端。
 */
#ifndef __TFLM_MODEL_PROCESS_LQ_HPP__
#define __TFLM_MODEL_PROCESS_LQ_HPP__

#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <net.h> // ncnn 头文件

/**
 * @brief 单次分类推理结果
 */
struct LQ_InferenceResult {
    int class_index;        ///< 类别索引，推理失败时为 -1
    std::string label;      ///< 类别名称，推理失败时为 "None"
    float confidence;       ///< softmax 后的置信度
};

/**
 * @brief ncnn 分类模型封装
 * @details 负责模型加载、图像预处理、前向推理和结果后处理。
 */
class LQ_NCNN {
public:
    LQ_NCNN();
    ~LQ_NCNN();

    /**
     * @brief 初始化 ncnn 模型
     * @param param_path ncnn param 文件路径
     * @param bin_path ncnn bin 权重文件路径
     * @param input_width 模型输入宽度
     * @param input_height 模型输入高度
     * @return true 模型加载成功，false 加载失败
     * @note 输入节点名和输出节点名需要与 infer() 中的 "in0" / "out0" 保持一致。
     */
    bool init(const std::string& param_path, const std::string& bin_path,int input_width, int input_height);

    /**
     * @brief 执行一次分类推理
     * @param bgr_image 输入 BGR 图像，通常来自 OpenCV Mat
     * @return LQ_InferenceResult 分类结果
     * @note 如果未初始化或输入为空，会返回 class_index=-1 的无效结果。
     */
    LQ_InferenceResult infer(const cv::Mat& bgr_image) const;

private:
    /**
     * @brief 从输出向量中寻找最大概率类别
     * @param logits softmax 后的输出向量
     * @param prob 输出参数，保存最大概率值
     * @return 最大值所在索引，输入为空时返回 -1
     */
    static int argmax(const ncnn::Mat& logits, float& prob);

private:
    ncnn::Net net_;                    ///< ncnn 网络对象
    std::vector<std::string> labels_;  ///< 类别标签表，索引必须与训练时类别顺序一致
    bool initialized_ = false;         ///< 模型是否已经成功初始化

    int kInputWidth = 40;              ///< 模型输入宽度，需与训练时保持一致
    int kInputHeight = 40;             ///< 模型输入高度，需与训练时保持一致

    const float kMeanVals[3] = {123.675f, 116.28f, 103.53f};      ///< ImageNet 均值，RGB 顺序
    const float kNormVals[3] = {0.01712475f, 0.017507f, 0.01742919f}; ///< ImageNet 标准差倒数，ncnn 公式为 (x - mean) * norm
};

#endif // __TFLM_MODEL_PROCESS_LQ_HPP__
