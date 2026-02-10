#pragma once

#include <iostream>
#include <ros/ros.h>
#include <robot_msgs/MotorCommand.h>
#include <robot_msgs/MotorState.h>
#include <sensor_msgs/Imu.h>
#include <boost/bind.hpp>
#include <thread>
#include <vector>

#include "hardware_interface.hpp"

class gazebo_interface : public hardware_interface
{
private:
    ros::NodeHandle _nh;

    std::vector<ros::Subscriber> _sub;
    std::vector<ros::Publisher> _pub;

    ros::Subscriber _imu_sub;

    std::thread _gazebo_control_thread;

    std::vector<std::string> _rec_topic;
    std::vector<std::string> _pub_topic;

    std::vector<std::string>
        imu_topic = {
            "/base_imu"};

public:
    gazebo_interface(const std::string &hardware_name = "gazebo_interface") : hardware_interface(hardware_name)
    {
        running = true;
        data_received = false;

        _sub.resize(param.dof_nums);
        _pub.resize(param.dof_nums);

        for (size_t i = 0; i < param.joint_name.size(); i++)
        {
            std::string base_topic = "/" + param.robot_name + "_gazebo/" + param.joint_name[i] + "_controller/";
            _rec_topic.push_back(base_topic + "state");
            _pub_topic.push_back(base_topic + "command");
        }

        for (int i = 0; i < param.dof_nums; i++)
        {
            _sub[i] = _nh.subscribe<robot_msgs::MotorState>(_rec_topic[i], 10, boost::bind(&gazebo_interface::motor_state_callback, this, _1, i));
            _pub[i] = _nh.advertise<robot_msgs::MotorCommand>(_pub_topic[i], 10);
        }
        _imu_sub = _nh.subscribe<sensor_msgs::Imu>(imu_topic[0], 10, &gazebo_interface::imu_callback, this);

        _gazebo_control_thread = std::thread(&gazebo_interface::gazebo_control, this);
    }

    void motor_state_callback(const robot_msgs::MotorState::ConstPtr &msg, int index)
    {
        data_received = true;
        pos[index] = (*msg).q;
        vel[index] = (*msg).dq;
    }

    void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        std::vector<float> quaternion = {static_cast<float>((*msg).orientation.w), static_cast<float>((*msg).orientation.x), static_cast<float>((*msg).orientation.y), static_cast<float>((*msg).orientation.z)};
        gravity_projection = QuatRotateInverse(quaternion, {0.0, 0.0, -1.0});
        angle_vel = {static_cast<float>((*msg).angular_velocity.x), static_cast<float>((*msg).angular_velocity.y), static_cast<float>((*msg).angular_velocity.z)};
    }

    void cmd_publish()
    {
        for (int i = 0; i < param.dof_nums; i++)
        {
            robot_msgs::MotorCommand cmd;
            cmd.q = cmd_pos[i];
            cmd.dq = cmd_vel[i];
            cmd.tau = cmd_torque[i];
            cmd.kp = kp[i];
            cmd.kd = kd[i];
            _pub[i].publish(cmd);
        }
    }

    void gazebo_control()
    {
        ros::Rate rate(param.hardware_rate);
        while (running)
        {
            cmd_publish();
            ros::spinOnce();
            rate.sleep();
        }
    }

    ~gazebo_interface()
    {
        if (_gazebo_control_thread.joinable())
        {
            _gazebo_control_thread.join();
        }
        running = false;
    }
};
