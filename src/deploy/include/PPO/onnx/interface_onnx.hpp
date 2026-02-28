#pragma once

#include "../rl_data_operatation.hpp"
#include "iostream"
#include "vector"
#include <onnxruntime_cxx_api.h>
#include <cmath>

class InterfaceOnnx : public RLDataOperation
{
private:
    std::string _model_path; // 模型的文件路径
    std::string _device;     // 模型推理的硬件平台，包含CPU或者GPU

    // ONNX Runtime相关成员
    Ort::Env _env;                        // onnxruntime环境
    Ort::SessionOptions _session_options; // 会话选项
    Ort::MemoryInfo _memory_info;         // onnxruntime内存信息
    Ort::Session _session;                // onnxruntime会话

    float *action_data;

    std::vector<std::string> _input_names_str;  // 输入节点名称字符串
    std::vector<std::string> _output_names_str; // 输出节点名称字符串
    std::vector<const char *> _input_names;     // 输入节点名称指针
    std::vector<const char *> _output_names;    // 输出节点名称指针

public:
    InterfaceOnnx(const std::string &model_path); // 第一个参数是模型的文件路径，第二个参数是模型推理的硬件平台，包含CPU或者GPU
    ~InterfaceOnnx() noexcept;

    void push_forward() override;
};

InterfaceOnnx::InterfaceOnnx(const std::string &model_path)
    : _model_path(model_path),
      _device("CPUExecutionProvider"),
      _env(ORT_LOGGING_LEVEL_WARNING, "policy_forward"),
      _session_options(),
      _memory_info(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
      _session(nullptr)
{
    // 设置会话选项
    _session_options.SetIntraOpNumThreads(1);
    _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    
    try {
        _session = Ort::Session(_env, _model_path.c_str(), _session_options);
    } catch (const std::exception& e) {
        std::cerr << "ERROR: Failed to load ONNX model: " << e.what() << std::endl;
        throw;
    }

    // 安全地存储节点名称字符串
    _input_names_str.push_back(data.obs_group);
    _output_names_str.push_back(data.act_group);
    
    // 使用持久字符串的指针
    _input_names.push_back(_input_names_str[0].c_str());
    _output_names.push_back(_output_names_str[0].c_str());

    // 测试ONNX模型是否加载成功，进行一次推理，如果推理存在问题就会直接报错，如果没有，就会正常输出，表明模型加载成功
    std::vector<float> dummy_input = std::vector<float>(param.obs_dim, 1.0);
    std::array<int64_t, 2> input_shape{1, param.obs_dim};

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        _memory_info, dummy_input.data(), param.obs_dim, input_shape.data(), input_shape.size());

    auto reaction = _session.Run(Ort::RunOptions{nullptr},
                                 _input_names.data(), &input_tensor, 1,
                                 _output_names.data(), 1);

    std::cout << "ONNX policy network test success" << std::endl;
}

InterfaceOnnx::~InterfaceOnnx()
{
    // 显式释放ONNX资源
    try {
        // 清空名称向量
        _input_names.clear();
        _output_names.clear();
        _input_names_str.clear();
        _output_names_str.clear();
        
        // 释放会话（RAII会自动处理，但显式重置更安全）
        _session = Ort::Session(nullptr);
    } catch (const std::exception& e) {
        std::cerr << "WARNING: Exception in ONNX destructor: " << e.what() << std::endl;
    }
}

void InterfaceOnnx::push_forward() // 前向推理，最后输出的是动作值
{
    try
    {
        // 获取观测数据
        std::vector<float> current_obs = set_obs();

        // 安全检查：观测数据
        if (current_obs.empty() || current_obs.size() != param.obs_dim)
        {
            std::cerr << "ERROR: Invalid observation data! Expected: "
                      << param.obs_dim << ", Got: " << current_obs.size() << std::endl;
            return;
        }

        std::array<int64_t, 2> input_shape{1, param.obs_dim};

        // 创建输入张量
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            _memory_info,
            current_obs.data(), current_obs.size(),
            input_shape.data(), input_shape.size());

        std::vector<Ort::Value> inputs;
        inputs.emplace_back(std::move(input_tensor));

        // 安全检查：节点名称指针
        if (_input_names.empty() || _output_names.empty()) {
            std::cerr << "ERROR: Input/Output names not initialized!" << std::endl;
            return;
        }
        
        // 执行推理
        auto output_tensors = _session.Run(Ort::RunOptions{nullptr},
                                           _input_names.data(),
                                           inputs.data(), 1,
                                           _output_names.data(), 1);

        // 安全检查：输出张量
        if (output_tensors.empty())
        {
            std::cerr << "ERROR: No output tensors returned!" << std::endl;
            return;
        }

        // 立即提取数据，避免悬空指针
        float *temp_action_data = output_tensors[0].GetTensorMutableData<float>();

        // 安全检查：数据指针
        if (temp_action_data == nullptr)
        {
            std::cerr << "ERROR: Null action data pointer!" << std::endl;
            return;
        }

        // 获取输出张量信息
        auto tensor_info = output_tensors[0].GetTensorTypeAndShapeInfo();
        size_t num_elements = tensor_info.GetElementCount();

        // 安全检查：输出维度
        if (num_elements != param.act_dim)
        {
            std::cerr << "WARNING: Action dimension mismatch! Expected: "
                      << param.act_dim << ", Got: " << num_elements << std::endl;
        }

        // 安全拷贝数据
        std::vector<float> action(temp_action_data, temp_action_data + num_elements);

        // 存储动作数据
        data.obs.last_action = action;

        // 更新动作数据
        set_act(action);
    }
    catch (const std::exception &e)
    {
        std::cerr << "ONNX inference error: " << e.what() << std::endl;
    }
}