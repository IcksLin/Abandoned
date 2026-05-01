/**
 * @file tflm_model_process_lq.cpp
 * @brief 基于 ncnn 的目标分类推理实现
 */
#include "tflm_model_process_lq.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <iostream>

/**
 * @brief 构造分类器并初始化类别表
 * @note 类别顺序必须与模型训练时的输出顺序一致。
 */
LQ_NCNN::LQ_NCNN() : initialized_(false) {
    labels_ = {"material", "traffic", "weapon"};

}

/**
 * @brief 析构时释放 ncnn 网络资源
 */
LQ_NCNN::~LQ_NCNN() {
    net_.clear();
}

/**
 * @brief 加载 ncnn 模型参数和权重
 * @param param_path param 文件路径
 * @param bin_path bin 文件路径
 * @param input_width 模型输入宽度
 * @param input_height 模型输入高度
 * @return true 加载成功，false 加载失败
 */
bool LQ_NCNN::init(const std::string& param_path, const std::string& bin_path,int input_width, int input_height) {
    kInputWidth = input_width;
    kInputHeight = input_height;

    net_.opt.use_vulkan_compute = false;
    net_.opt.num_threads = 1; // 2K0300 建议单线程或根据核心数调整
    net_.opt.use_fp16_packed = true;
    net_.opt.use_fp16_storage = true;
    net_.opt.use_fp16_arithmetic = true;

    if (net_.load_param(param_path.c_str()) != 0) {
        return false;
    }
    if (net_.load_model(bin_path.c_str()) != 0) {
        return false;
    }

    initialized_ = true;
    return true;
}

/**
 * @brief 执行一次 BGR 图像分类推理
 * @param bgr_image 输入图像
 * @return 分类结果，失败时 class_index 为 -1
 */
LQ_InferenceResult LQ_NCNN::infer(const cv::Mat& bgr_image) const {
    LQ_InferenceResult res = {-1, "None", 0.0f};

    if (!initialized_ || bgr_image.empty()) return res;

    // 1. 预处理：Resize
    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(kInputWidth, kInputHeight));

    // 2. 转换为 ncnn::Mat 并进行色彩空间转换 (BGR -> RGB) 与归一化
    // 静态链接 libncnn.a 时，此函数会自动包含必要的转换算子
    ncnn::Mat input = ncnn::Mat::from_pixels(resized.data, 
                                            ncnn::Mat::PIXEL_BGR2RGB, 
                                            kInputWidth, kInputHeight);
    
    input.substract_mean_normalize(kMeanVals, kNormVals);

    // 3. 执行推理
    ncnn::Extractor ex = net_.create_extractor();
    ex.input("in0", input); // 确保与你模型中的输入节点名一致

    ncnn::Mat out;
    ex.extract("out0", out); // 确保与你模型中的输出节点名一致

    // 4. 后处理：Softmax 归一化（使输出在 0~1 之间，方便置信度过滤）
    // 如果你的模型最后自带了 Softmax 层，可以跳过这一步
    ncnn::Layer* softmax = ncnn::create_layer("Softmax");
    ncnn::ParamDict pd;
    softmax->load_param(pd);
    softmax->forward_inplace(out, net_.opt);
    delete softmax;

    // 5. 获取结果
    float prob = 0.0f;
    int class_id = argmax(out, prob);

    if (class_id >= 0 && class_id < (int)labels_.size()) {
        res.class_index = class_id;
        res.label = labels_[class_id];
        res.confidence = prob;
    }

    return res;
}

/**
 * @brief 获取输出向量中的最大概率类别
 * @param logits softmax 后的类别概率
 * @param prob 输出参数，返回最大概率
 * @return 最大概率类别索引
 */
int LQ_NCNN::argmax(const ncnn::Mat& logits, float& prob) {
    if (logits.empty()) return -1;

    int best_index = 0;
    float max_prob = -1.0f;

    const float* ptr = logits;
    for (int i = 0; i < logits.w; i++) {
        if (ptr[i] > max_prob) {
            max_prob = ptr[i];
            best_index = i;
        }
    }
    prob = max_prob;
    return best_index;
}
