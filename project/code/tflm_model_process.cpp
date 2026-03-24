#include "tflm_model_process.hpp"
#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include <chrono>

// 1. 静态成员变量的定义与内存对齐（必须放在类外全局作用域）
alignas(16) uint8_t TFLMModelProcessor::tensor_arena[TENSOR_ARENA_SIZE];

// 2. 构造函数：删掉对 tensor_arena 的初始化，因为它现在是数组名，不是指针
TFLMModelProcessor::TFLMModelProcessor() : interpreter(nullptr), model(nullptr) {
    // 这里不需要 new 任何东西
}

// 3. 析构函数：删掉 delete[] tensor_arena
TFLMModelProcessor::~TFLMModelProcessor() {
    if (interpreter) {
        delete interpreter;
        interpreter = nullptr;
    }
    // tensor_arena 是静态分配的，生命周期随类或程序，不需要手动释放
}
bool TFLMModelProcessor::init(const unsigned char* model_data) {
    static tflite::MicroErrorReporter error_reporter;
    tflite::InitializeTarget();

    model = tflite::GetModel(model_data);
    
    // 【修改点 3】: 算子预警
    // 既然你已经链接了 unpack.cc，确保 AddUnpack 存在。
    // 如果运行时还崩，请尝试注释掉 AddPack/AddSlice 等你还没下载 .cc 的算子
    // 在 init 函数中，暂时只保留这些，看看能不能跑过 AllocateTensors
    static tflite::MicroMutableOpResolver<30> resolver;
    // --- 基础算子 ---
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddSoftmax();
    resolver.AddReshape();
    resolver.AddFullyConnected();
    resolver.AddConcatenation();

    // --- 针对 GhostTiny 的必须算子 (根据你的 DEBUG 日志补全) ---
    resolver.AddAdd();            // 对应日志中的 ADD
    resolver.AddStridedSlice();   // 对应日志中的 STRIDED_SLICE (非常多!)
    resolver.AddMean();           // 对应日志中的 MEAN
    resolver.AddLogistic();       // 对应日志中的 LOGISTIC
    resolver.AddMul();            // 对应日志中的 MUL
    resolver.AddPack();           // 对应日志中的 PACK
    resolver.AddShape();          // 对应日志中的 SHAPE

    // --- 补充 (虽然日志最后几行没显示，但 ReLU 通常被合并或单独存在) ---
    resolver.AddRelu();

    // 【关键检查点】: 打印一下 arena 地址，看看是不是以 0 结尾（16字节对齐）
    printf("DEBUG: Tensor Arena Address: %p\n", (void*)tensor_arena);

    interpreter = new tflite::MicroInterpreter(
        model, resolver, tensor_arena, TENSOR_ARENA_SIZE
    );

    // if (interpreter->AllocateTensors() != kTfLiteOk) {
    //     TF_LITE_REPORT_ERROR(&error_reporter, "❌ AllocateTensors 失败!");
    //     return false;
    // }

    printf("DEBUG: 正在检查模型所需的算子...\n");
    auto* subgraphs = model->subgraphs();
    for (unsigned int i = 0; i < subgraphs->size(); ++i) {
        auto* operators = (*subgraphs)[i]->operators();
        for (unsigned int j = 0; j < operators->size(); ++j) {
            int op_idx = (*operators)[j]->opcode_index();
            auto* opcode = (*model->operator_codes())[op_idx];
            
            // 打印出每一个算子的类型 ID
            if (opcode->builtin_code() == tflite::BuiltinOperator_CUSTOM) {
                printf("  - 需要自定义算子: %s\n", opcode->custom_code()->c_str());
            } else {
                printf("  - 需要内置算子 ID: %d (名称: %s)\n", 
                        (int)opcode->builtin_code(), 
                        tflite::EnumNameBuiltinOperator(opcode->builtin_code()));
            }
        }
    }

    printf("DEBUG: 准备执行 AllocateTensors...\n");
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        printf("❌ 分配失败，状态码: %d\n", (int)allocate_status);
        return false;
    }
    printf("✅ 分配成功！\n");

    TF_LITE_REPORT_ERROR(&error_reporter, "✅ 成功分配！Arena 使用量: %d 字节", 
                         interpreter->arena_used_bytes());
    
    is_initialized = true;
    return true;
}

InferenceResult TFLMModelProcessor::process(const cv::Mat& src_img) {
    InferenceResult res = {-1, "Unknown", 0.0f, 0};

    if (!is_initialized || src_img.empty()) return res;

    // 1. 预处理：缩放 -> 归一化 -> 色彩转换
    cv::Mat resized_img, float_img;
    cv::resize(src_img, resized_img, cv::Size(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT));
    resized_img.convertTo(float_img, CV_32FC3, 1.0 / 255.0); // 注意：此处根据训练习惯决定是否/255
    cv::cvtColor(float_img, float_img, cv::COLOR_BGR2RGB);

    if (!float_img.isContinuous()) float_img = float_img.clone();

    // 2. 填充模型输入
    float* input_data = interpreter->input(0)->data.f;
    memcpy(input_data, float_img.ptr<float>(0), MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CHANNEL * sizeof(float));

    // 3. 执行推理并计时
    auto start = std::chrono::high_resolution_clock::now();
    interpreter->Invoke();
    auto end = std::chrono::high_resolution_clock::now();
    res.inference_time = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    // 4. 解析输出
    float* output_data = interpreter->output(0)->data.f;
    int max_idx = 0;
    float max_val = output_data[0];

    for (int i = 1; i < MODEL_OUTPUT_CLASS_NUM; i++) {
        if (output_data[i] > max_val) {
            max_val = output_data[i];
            max_idx = i;
        }
    }

    res.class_index = max_idx;
    res.label = class_labels[max_idx];
    res.confidence = max_val;

    return res;
}


