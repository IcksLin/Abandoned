//本头源文件移植集成逐飞推理例程
#ifndef TFLM_MODEL_PROCESS_H
#define TFLM_MODEL_PROCESS_H

#include <opencv2/opencv.hpp>
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"


// --- 模型相关硬参数 ---
#define MODEL_INPUT_WIDTH          80
#define MODEL_INPUT_HEIGHT         80
#define MODEL_INPUT_CHANNEL        3
#define MODEL_OUTPUT_CLASS_NUM     3
#define TFLITE_OP_RESOLVER_MAX_NUM 20
#define TENSOR_ARENA_SIZE          (1024 * 1024)

// 推理结果结构体
struct InferenceResult {
    int class_index;       // 类别索引
    const char* label;     // 类别名称
    float confidence;      // 置信度
    long inference_time;   // 推理耗时(us)
};

// --- 修改后的类定义片段 ---
class TFLMModelProcessor {
public:
    TFLMModelProcessor();
    ~TFLMModelProcessor();

    bool init(const unsigned char* model_data);
    InferenceResult process(const cv::Mat& src_img);

private:
    const char* class_labels[MODEL_OUTPUT_CLASS_NUM] = {"materials", "traffic", "weapon"};
    
    // 【修改点 1】: 使用 alignas(16) 确保 16 字节对齐
    // 将它声明为静态或直接放在类中（如果类对象也是全局单例的话）
    // 为了保险，我们在这里定义，并在 cpp 中实现
    static uint8_t tensor_arena[TENSOR_ARENA_SIZE] alignas(16); 
    
    tflite::MicroInterpreter* interpreter;
    const tflite::Model* model;
    bool is_initialized = false;
};

#endif