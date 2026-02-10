#pragma once

#include <iostream>
#include <ros/ros.h>
#include <robot_msgs/MotorCommand.h>
#include <robot_msgs/MotorState.h>
#include <robot_msgs/RobotState.h>
#include <robot_msgs/RobotCommand.h>
#include <sensor_msgs/Imu.h>
#include <boost/bind.hpp>
#include <thread>
#include <vector>
#include "robot_param.hpp"

#include "hardware_interface.hpp"

class mujoco_interface : public hardware_interface
{
private:
    ros::NodeHandle _nh;
    ros::Publisher robot_command_pub;
    ros::Subscriber robot_state_sub;

    std::thread mujoco_control_thread;

    std::vector<std::string> robot_State_topic;

    std::vector<std::string> robot_Command_topic;

    robot_msgs::RobotCommand robot_command;

public:
    mujoco_interface(const std::string &hardware_name="mujoco_interface") : hardware_interface(hardware_name)
    {
        running = true;
        data_received = false;

        robot_command.motor_command.resize(param.dof_nums);

        robot_State_topic = {"/" + param.robot_name + "_mujoco/robot_state"};
        robot_Command_topic = {"/" + param.robot_name + "_mujoco/robot_command"};


        robot_command_pub = _nh.advertise<robot_msgs::RobotCommand>(robot_Command_topic[0], 10);
        robot_state_sub = _nh.subscribe<robot_msgs::RobotState>
            (robot_State_topic[0], 10, &mujoco_interface::robot_state_callback, this);

        mujoco_control_thread = std::thread(&mujoco_interface::mujoco_control, this);

    }

    void robot_state_callback(const robot_msgs::RobotState::ConstPtr &msg)
    {
        data_received = true;
  
        for (int i = 0; i < param.dof_nums; i++)
        {
            pos[i] = (*msg).motor_state[i].q; // 位置
            vel[i] = (*msg).motor_state[i].dq; // 速度
        }

        std::vector<float> quaternion = {(*msg).imu.quaternion[0], (*msg).imu.quaternion[1], (*msg).imu.quaternion[2], (*msg).imu.quaternion[3]};
        gravity_projection = QuatRotateInverse(quaternion, {0.0, 0.0, -1.0});
        angle_vel = {(*msg).imu.gyroscope[0], (*msg).imu.gyroscope[1], (*msg).imu.gyroscope[2]};
    }

    void cmd_publish()
    {
        for (int i = 0; i < param.dof_nums; i++)
        {
            robot_command.motor_command[i].q = cmd_pos[i]; // 位置
            robot_command.motor_command[i].dq = cmd_vel[i]; // 速度
            robot_command.motor_command[i].kp = kp[i]; // 位置比例增益
            robot_command.motor_command[i].kd = kd[i]; // 速度微分增益
            robot_command.motor_command[i].tau = cmd_torque[i]; // 力矩
        }
        robot_command_pub.publish(robot_command);
    }

    void mujoco_control()
    {
        ros::Rate loop_rate(param.hardware_rate);
        while (running)
        {
            cmd_publish();
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
    
    ~mujoco_interface()
    {
        running = false;
        if (mujoco_control_thread.joinable())
            mujoco_control_thread.join();
    }
};
