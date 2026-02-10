#pragma once

#include <map>
#include <string>
#include <vector>
#include <iostream>

class robot_control_param
{
public:
    const std::string robot_name = "m20"; // 机器人名称
    const int hardware_rate = 500;        // 硬件循环频率
    const int control_rate = 500;         // 控制循环频率

    /*====================================================base_parameters============================================================= */
    const float Safety_Factor = 0.9; // 安全系数因子

    // 关节极限参数
    const float Max_torque_joint = 76.4 * Safety_Factor;
    const float Max_speed_joint = 22.4 * Safety_Factor;
    const float Max_torque_wheel = 21.6 * Safety_Factor;
    const float Max_speed_wheel = 79.3 * Safety_Factor;

    std::vector<std::string> joint_name = { // 关节名称
        "fl_hipx", "fl_hipy", "fl_knee",
        "fr_hipx", "fr_hipy", "fr_knee",
        "hl_hipx", "hl_hipy", "hl_knee",
        "hr_hipx", "hr_hipy", "hr_knee",
        "fl_wheel", "fr_wheel", "hl_wheel", "hr_wheel"};

    const int dof_nums = joint_name.size();

    std::map<std::string, std::vector<float>> joint_limit = {
        {"hipx", {-0.436, 0.611}},
        {"hipy", {-2.74, 2.583}},
        {"knee", {-2.809, 2.792}},
    };

    std::vector<float> dof_pos_robot = {
        0.0, 0.60, -1.0,
        0.0, 0.60, -1.0,
        0.0, -0.60, 1.0,
        0.0, -0.60, 1.0,
        0.0, 0.0, 0.0, 0.0};

    std::vector<float> kp_fix = {
        100.0, 100.0, 100.0,
        100.0, 100.0, 100.0,
        100.0, 100.0, 100.0,
        100.0, 100.0, 100.0,
        0.0, 0.0, 0.0, 0.0};

    std::vector<float> kd_fix = {
        3.0, 3.0, 3.0,
        3.0, 3.0, 3.0,
        3.0, 3.0, 3.0,
        3.0, 3.0, 3.0,
        2.0, 2.0, 2.0, 2.0};

    /*====================================================rl_parameters============================================================= */

    float dt = 0.005;   // 时间步长
    int decimation = 4; // 动作执行间隔

    float _omega_scale = 0.25;   // 角速度缩放因子
    float _dof_pos_scale = 1;    // 关节位置缩放因子
    float _dof_vel_scale = 0.05; // 关节速度缩放因子
    float _command_scale = 1.0;  // 控制命令缩放因子

    std::vector<float> dof_pos_rl = {
        0.00, 0.60, -1.0,
        0.00, 0.60, -1.0,
        0.00, -0.60, 1.0,
        0.00, -0.60, 1.0,
        0.0, 0.0, 0.0, 0.0};

    std::vector<float> kp_rl = {
        80.0, 80.0, 80.0,
        80.0, 80.0, 80.0,
        80.0, 80.0, 80.0,
        80.0, 80.0, 80.0,
        0.0, 0.0, 0.0, 0.0};

    std::vector<float> kd_rl = {
        2.0, 2.0, 2.0,
        2.0, 2.0, 2.0,
        2.0, 2.0, 2.0,
        2.0, 2.0, 2.0,
        0.6, 0.6, 0.6, 0.6};

    std::vector<float> action_scale_robot = {0.125, 0.25, 0.25,
                                             0.125, 0.25, 0.25,
                                             0.125, 0.25, 0.25,
                                             0.125, 0.25, 0.25,
                                             5.0, 5.0, 5.0, 5.0};

    std::map<std::string, int> observation_group = {
        {"angular_vel", 3},
        {"gravity_projection", 3},
        {"command", 3},
        {"joint_pos", dof_nums},
        {"joint_vel", dof_nums},
        {"last_action", dof_nums}};

    std::map<std::string, int> action_group = {
        {"joint_pos", 12},
        {"joint_vel", 4}};

    const int obs_dim = observation_group["angular_vel"] +
                        observation_group["gravity_projection"] +
                        observation_group["command"] +
                        observation_group["joint_pos"] +
                        observation_group["joint_vel"] +
                        observation_group["last_action"];
                        
    const int act_dim = action_group["joint_pos"] + action_group["joint_vel"];
};
