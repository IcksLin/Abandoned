#include "tflm_model_process.hpp"
#include <chrono>
TFLMModelProcessor::TFLMModelProcessor() : tensor_arena(nullptr), interpreter(nullptr), model(nullptr) {
    tensor_arena = new uint8_t[TENSOR_ARENA_SIZE];
}

TFLMModelProcessor::~TFLMModelProcessor() {
    if (interpreter) delete interpreter;
    if (tensor_arena) delete[] tensor_arena;
}

bool TFLMModelProcessor::init(const unsigned char* model_data) {
    tflite::InitializeTarget();

    model = tflite::GetModel(model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        printf("❌ 模型版本不匹配!\n");
        return false;
    }

    // 静态局部变量或类成员以保证生命周期
    static tflite::MicroMutableOpResolver<TFLITE_OP_RESOLVER_MAX_NUM> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddMaxPool2D();
    resolver.AddFullyConnected();
    resolver.AddRelu6();
    resolver.AddSoftmax();
    resolver.AddReshape();
    // ... 根据需要添加其他算子 ...

    interpreter = new tflite::MicroInterpreter(model, resolver, tensor_arena, TENSOR_ARENA_SIZE);
    
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        printf("❌ 分配张量空间失败!\n");
        return false;
    }

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