#pragma once

#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
#include <unistd.h>
#include "utils.hpp"
#include "FSM_State.hpp"
#include <robot_msgs/RobotState.h>
#include <robot_msgs/RobotCommand.h>
#include "PPO/onnx/interface_onnx.hpp"
#include "data_stream/data_stream.hpp"
#include "PPO/Torchscript/interface_torchscript.hpp"
#include "Timer/thread_Timer.hpp"

#define YELLOW "\033[33m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define BLUE "\033[34m"
#define WHITE "\033[0m"
#define RESET "\033[0m"

/*============================================================失能状态机======================================================================*/
class FSM_Disable : public FSM_State
{
private:
public:
    FSM_Disable(const std::string &state_name, hardware_interface *interface) : FSM_State(state_name, interface) {

                                                                                };
    ~FSM_Disable() = default;

    void enter() override
    {
        std::cout << YELLOW << "\nEnter Disable state" << RESET << std::endl;
    }
    void run() override
    {
        _interface->command_set(_cmd[0], _cmd[1], _cmd[2], _cmd[3], _cmd[4]);
    }
    void exit() override
    {
        std::cout << GREEN << "Exit Disable state" << RESET << std::endl;
    }
};

/*============================================================阻尼状态机======================================================================*/
class FSM_Passive : public FSM_State
{
private:
public:
    FSM_Passive(const std::string &state_name, hardware_interface *interface) : FSM_State(state_name, interface)
    {
        _cmd[4] = std::vector<float>(param.dof_nums, 6.0f); // kd指令
    };
    ~FSM_Passive() = default;

    void enter() override
    {
        std::cout << YELLOW << "\nEnter Passive state" << RESET << std::endl;
    }
    void run() override
    {
        _interface->command_set(_cmd[0], _cmd[1], _cmd[2], _cmd[3], _cmd[4]);
    }
    void exit() override
    {
        std::cout << GREEN << "Exit Passive state" << RESET << std::endl;
    }
};

/*============================================================站起状态机======================================================================*/
class FSM_Getup : public FSM_State
{
private:
    const int pre_during = getCycleCount(1.2, param.control_rate);
    const int stand_during = getCycleCount(1.2, param.control_rate);

    int count = 0;
    int stage = 0;

    int version = 0;

    std::vector<float> _Init_state;

    std::vector<float> pre_pos = {
        0.0, 1.35, -2.7,
        0.0, 1.35, -2.7,
        0.0, -1.35, 2.7,
        0.0, -1.35, 2.7,
        0.0, 0.0, 0.0, 0.0};

public:
    FSM_Getup(const std::string &state_name, hardware_interface *interface, RobotState_List *previous_state) : FSM_State(state_name, interface)
    {
        _Init_state.resize(param.dof_nums);
        _previous_state = previous_state;

        _cmd[3] = param.kp_fix; // kp指令
        _cmd[4] = param.kd_fix; // kd指令
    };
    ~FSM_Getup() = default;

    void enter() override
    {
        std::cout << YELLOW << "\nEnter Getup state" << RESET << std::endl;
        _Init_state = _interface->get_joint_pos();

        count = 0;
        stage = 0;

        if (*_previous_state == RobotState_List::Passive || *_previous_state == RobotState_List::Getdown)
            version = 1;
        else
            version = 2;
    }

    void run() override
    {
        switch (version)
        {
        case 1: // 分段差分
            if (stage == 0)
            {
                if (calculate_target_joint_pos(_cmd[0], _Init_state, pre_pos, count, pre_during))
                {
                    count = 0;
                    stage++;
                }
            }
            else
                calculate_target_joint_pos(_cmd[0], pre_pos, param.dof_pos_robot, count, stand_during);
            break;
        case 2: // 直接差分
            calculate_target_joint_pos(_cmd[0], _Init_state, param.dof_pos_robot, count, stand_during);
            break;
        default:
            break;
        }

        _interface->command_set(_cmd[0], _cmd[1], _cmd[2], _cmd[3], _cmd[4]);
    }

    void exit() override
    {
        std::cout << GREEN << "Exit Getup state" << RESET << std::endl;
    }
};

/*============================================================蹲下状态机======================================================================*/
class FSM_Getdown : public FSM_State
{
private:
    const int pre_during = getCycleCount(1.2, param.control_rate);
    const int getdown_during = getCycleCount(1.2, param.control_rate);

    int count = 0;
    int stage = 0;

    std::vector<float> _Init_state;

    std::vector<float> final_getdown_pos = {
        0.0, 1.35, -2.7,
        0.0, 1.35, -2.7,
        0.0, -1.35, 2.7,
        0.0, -1.35, 2.7,
        0.0, 0.0, 0.0, 0.0};

public:
    FSM_Getdown(const std::string &state_name, hardware_interface *interface) : FSM_State(state_name, interface)
    {
        _Init_state.resize(param.dof_nums);

        _cmd[3] = param.kp_fix; // kp指令
        _cmd[4] = param.kd_fix; // kd指令
    };
    ~FSM_Getdown() = default; // 析构函数

    void enter() override
    {
        std::cout << YELLOW << "\nEnter Getdown state" << RESET << std::endl;
        _Init_state = _interface->get_joint_pos();
        count = 0;
        stage = 0;
    }

    void run() override
    {
        switch (stage)
        {
        case 0:
            if (calculate_target_joint_pos(_cmd[0], _Init_state, param.dof_pos_robot, count, pre_during))
            {
                count = 0;
                stage++;
            }
            break;
        case 1:
            if (calculate_target_joint_pos(_cmd[0], param.dof_pos_robot, final_getdown_pos, count, getdown_during))
            {
                count = 0;
                stage++;
            }
            break;
        default:
            break;
        }
        _interface->command_set(_cmd[0], _cmd[1], _cmd[2], _cmd[3], _cmd[4]);
    }

    void exit() override
    {
        std::cout << GREEN << "Exit Getdown state" << RESET << std::endl;
    }
};

/*============================================================强化学习运动控制状态机================================================================*/

class FSM_Rl_control : public FSM_State // 强化学习控制状态机
{
private:
    std::vector<float> *_command = nullptr;

    std::string _model_path;
    std::string _model_type;

    RLDataOperation *model_operation = nullptr;

    thread_Timer obs_collection;
    thread_Timer data_forward;

    int print_count = 0;
    std::vector<float> raw_action;

public:
    FSM_Rl_control(const std::string &state_name, hardware_interface *interface, const std::string &model_type, std::vector<float> *command)
        : FSM_State(state_name, interface),
          _model_type(model_type),
          _model_path(POLICY_MODEL_PATH + param.robot_name),
          _command(command)
    {
        // 初始化强化学习模型（使用RAII模式）
        try {
            if (_model_type == "onnx")
                model_operation = new InterfaceOnnx(_model_path + "/policy.onnx");
            else if (_model_type == "torch")
                model_operation = new InterfaceTorchscript(_model_path + "/policy.pt");
            else
                std::cout << "Error: Unsupported model type!" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "ERROR: Failed to initialize RL model: " << e.what() << std::endl;
            model_operation = nullptr;
        }

        obs_collection.start(1.0 / (param.dt),
                             std::bind(&RLDataOperation::observation_set, model_operation,
                                       std::ref(_interface->angle_vel), std::ref(_interface->gravity_projection), std::cref(*_command), std::ref(_interface->pos), std::ref(_interface->vel)),
                             "Obs_Collection_Timer");

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        data_forward.start(1.0 / (param.dt * param.decimation), std::bind(&RLDataOperation::push_forward, model_operation), "Data_Forward_Timer");

        _cmd[3] = param.kp_rl; // kp指令
        _cmd[4] = param.kd_rl; // kd指令
    };

    void enter() override
    {
        std::cout << YELLOW << "\nEnter Rl_control state" << RESET << std::endl;
    }
    void run() override
    {
        raw_action = model_operation->get_act(); // 获取动作值

        for (int i = 0; i < param.action_group["action_pos"]; i++)
            _cmd[0][i] = raw_action[i];
        for (int i = 0; i < param.action_group["action_vel"]; i++)
            _cmd[1][i + param.action_group["action_pos"]] = raw_action[i + param.action_group["action_pos"]];

        _interface->command_set(_cmd[0], _cmd[1], _cmd[2], _cmd[3], _cmd[4]);

    }
    void exit() override
    {
        std::cout << GREEN << "Exit Rl_control state" << RESET << std::endl;
    }

    ~FSM_Rl_control()
    {
        obs_collection.stop();
        data_forward.stop();
        
        // 安全释放内存
        if (model_operation) {
            delete model_operation;
            model_operation = nullptr;
        }
    }
};