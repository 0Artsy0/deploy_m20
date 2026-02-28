#pragma once

#include <vector>
#include <numeric>
#include <iostream>
#include <stdexcept>
#include "utils.hpp"
#include "../robot_param.hpp"
#include "robot_msgs/RobotState.h"
#include "robot_msgs/RobotCommand.h"

struct observation
{
    std::vector<float> angular_vel;
    std::vector<float> gravity_projection;
    std::vector<float> command;
    std::vector<float> joint_pos;
    std::vector<float> joint_vel;
    std::vector<float> last_action;
};

struct action
{
    std::vector<float> action_pos;
    std::vector<float> action_vel;
};

struct rl_data
{
    std::string obs_group = "obs";     // 观测组
    std::string act_group = "actions"; // 动作组

    observation obs;
    action act;
};

class RLDataOperation
{
public:
    RLDataOperation() : param()
    {
        data.obs.angular_vel.resize(param.observation_group["angular_vel"], 0.0);
        data.obs.gravity_projection.resize(param.observation_group["gravity_projection"], 0.0);
        data.obs.command.resize(param.observation_group["command"], 0.0);
        data.obs.joint_pos.resize(param.observation_group["action_pos"], 0.0);
        data.obs.joint_vel.resize(param.observation_group["action_vel"], 0.0);
        data.obs.last_action.resize(param.observation_group["last_action"], 0.0);

        data.act.action_pos.resize(param.action_group["action_pos"], 0.0);
        data.act.action_vel.resize(param.action_group["action_vel"], 0.0);
    };
    virtual ~RLDataOperation();

    rl_data data;
    robot_control_param param;

    std::vector<float> set_obs();
    void set_act(const std::vector<float> &act);

    std::vector<float> get_act();

    virtual void push_forward() = 0;

    void observation_set(std::vector<float> angular_vel, std::vector<float> gravity_projection, std::vector<float> command, std::vector<float> joint_angle, std::vector<float> joint_vel);
};
RLDataOperation::~RLDataOperation() = default;

std::vector<float> RLDataOperation::set_obs()
{
    std::vector<float> integrated_obs;

    for (size_t i = 0; i < param.observation_group["angular_vel"]; i++)
        integrated_obs.push_back(data.obs.angular_vel[i] * param._omega_scale);

    for (int i = 0; i < param.observation_group["gravity_projection"]; ++i)
        integrated_obs.push_back(data.obs.gravity_projection[i]);

    for (int i = 0; i < param.observation_group["command"]; ++i)
        integrated_obs.push_back(data.obs.command[i]);

    for (int i = 0; i < param.observation_group["joint_pos"] - 4; ++i) // ；轮子的位置全部设置为0
        integrated_obs.push_back((data.obs.joint_pos[i] - param.dof_pos_robot[i]) * param._dof_pos_scale);

    for (int i = 0; i < 4; ++i)
        integrated_obs.push_back(0);

    for (int i = 0; i < param.observation_group["joint_vel"]; ++i)
        integrated_obs.push_back(data.obs.joint_vel[i] * param._dof_vel_scale);

    for (int i = 0; i < param.observation_group["last_action"]; ++i)
        integrated_obs.push_back(data.obs.last_action[i]);

    data_clip(integrated_obs, 2);

    return integrated_obs;
}

void RLDataOperation::set_act(const std::vector<float> &act)
{
    std::vector<float> processed_joint;
    for (int i = 0; i < param.action_group["action_pos"]; ++i)
        processed_joint.push_back(act[i] * param.action_scale_robot[i] + param.dof_pos_robot[i]);
    data.act.action_pos = processed_joint;

    std::vector<float> processed_wheel;
    for (int i = 0; i < param.action_group["action_vel"]; ++i)
        processed_wheel.push_back(act[i + param.action_group["action_pos"]] * param.action_scale_robot[i + param.action_group["action_pos"]]);
    data.act.action_vel = processed_wheel;
}
std::vector<float> RLDataOperation::get_act()
{
    std::vector<float> act_cliped;

    for (auto &pos : data.act.action_pos)
        act_cliped.push_back(pos);
    for (auto &vel : data.act.action_vel)
        act_cliped.push_back(vel);

    data_clip(act_cliped, 2);

    return act_cliped;
}

void RLDataOperation::observation_set(std::vector<float> angular_vel, std::vector<float> gravity_projection, std::vector<float> command, std::vector<float> joint_pos, std::vector<float> joint_vel)
{
    data.obs.angular_vel = angular_vel;
    data.obs.gravity_projection = gravity_projection;
    data.obs.command = command;
    data.obs.joint_pos = joint_pos;
    data.obs.joint_vel = joint_vel;
}
