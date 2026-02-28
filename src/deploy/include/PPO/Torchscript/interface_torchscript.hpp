#pragma once

#include <cmath>
#include <vector>
#include "iostream"
#include <torch/utils.h>
#include <torch/script.h>
#include <ATen/Parallel.h>
#include <robot_msgs/RobotState.h>
#include "../rl_data_operatation.hpp"

class InterfaceTorchscript : public RLDataOperation
{
private:
    std::string _model_path; // 模型的文件路径
    std::string _device;     // 模型推理的硬件平台，包含CPU或者GPU

    torch::jit::script::Module model; // TorchScript模块

public:
    InterfaceTorchscript(const std::string &model_path); // 第一个参数是模型的文件路径，第二个参数是模型推理的硬件平台，包含CPU或者GPU
    ~InterfaceTorchscript() noexcept;

    void push_forward() override;
};

InterfaceTorchscript::InterfaceTorchscript(const std::string &model_path)
{
    try
    {
        // model = torch::jit::load(model_path, torch::Device(torch::kCPU));
        model = torch::jit::load(model_path);
        std::cout << "Successfully loaded Torch model" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cout << "Failed to load Torch model " << std::endl;
    }
}

InterfaceTorchscript::~InterfaceTorchscript() noexcept
{
}

void InterfaceTorchscript::push_forward() // 策略网络前向推理
{

    try
    {
        // Convert observation vector to Torch tensor
        std::vector<float> current_obs = set_obs();

        auto input_tensor = torch::tensor(current_obs, torch::kFloat32).reshape({1, static_cast<int64_t>(current_obs.size())});

        // Disable gradient computation before each forward pass
        torch::autograd::GradMode::set_enabled(false);

        // Ensure single-threaded execution (critical for performance!)
        torch::set_num_threads(1);

        // Execute forward inference
        auto output = model.forward({input_tensor}).toTensor();

        // Convert output tensor to vector
        auto output_accessor = output.accessor<float, 2>();

        // Ensure tensor is contiguous and on CPU
        auto cpu_tensor = output.is_contiguous() ? output : output.contiguous();
        if (cpu_tensor.device().type() != torch::kCPU)
        {
            cpu_tensor = cpu_tensor.to(torch::kCPU);
        }

        // Get data pointer and size
        float *data_ptr = cpu_tensor.data_ptr<float>();
        int64_t num_elements = cpu_tensor.numel();

        // Copy data to vector
        std::vector<float> act(data_ptr, data_ptr + num_elements);

        set_act(act); // 动作的缩放映射
        // Store action in data
        data.obs.last_action = act;
    }
    catch (const std::exception &e) // 捕获Torch推理过程中可能抛出的异常
    {
        std::cout << "Torch inference error: " << e.what() << std::endl;
    }
}
