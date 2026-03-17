#ifndef TFLM_MODEL_PROCESS_H
#define TFLM_MODEL_PROCESS_H

#include <opencv2/opencv.hpp>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"


// --- 模型相关硬参数 ---
#define MODEL_INPUT_WIDTH          40
#define MODEL_INPUT_HEIGHT         40
#define MODEL_INPUT_CHANNEL        3
#define MODEL_OUTPUT_CLASS_NUM     3
#define TFLITE_OP_RESOLVER_MAX_NUM 20
#define TENSOR_ARENA_SIZE          (128 * 1024)

// 推理结果结构体
struct InferenceResult {
    int class_index;       // 类别索引
    const char* label;     // 类别名称
    float confidence;      // 置信度
    long inference_time;   // 推理耗时(us)
};

class TFLMModelProcessor {
public:
    TFLMModelProcessor();
    ~TFLMModelProcessor();

    // 初始化：传入模型数据指针
    bool init(const unsigned char* model_data);

    // 推理接口：传入OpenCV矩阵，返回结构体
    InferenceResult process(const cv::Mat& src_img);

private:
    const char* class_labels[MODEL_OUTPUT_CLASS_NUM] = {"materials", "traffic", "weapon"};
    
    // TFLM 核心组件
    uint8_t* tensor_arena;
    tflite::MicroInterpreter* interpreter;
    const tflite::Model* model;
    
    bool is_initialized = false;
};

#endif