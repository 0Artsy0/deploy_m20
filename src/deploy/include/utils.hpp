#pragma once

#include <vector>
#include <iostream>
#include <robot_param.hpp>

bool calculate_target_joint_pos(std::vector<float> &pose,std::vector<float> &init_pose, std::vector<float> &target_pos,int &count,int during)//差分函数
{
    float percent=static_cast<float>(count)/static_cast<float>(during);

    if(init_pose.size()!=target_pos.size() || init_pose.size()!=pose.size())
    {
        std::cerr << "Error: Vector size mismatch! init_pose=" << init_pose.size() 
                  << ", target_pos=" << target_pos.size() << ", pose=" << pose.size() << std::endl;
        return false;
    }

    if(pose.size() < init_pose.size())
    {
        pose.resize(init_pose.size());
    }

    for(size_t i=0;i<init_pose.size();i++)//差值处理
    {
        pose[i]=init_pose[i]*(1.0f - percent)+target_pos[i]*percent;
    }

    if(count <= during)
    {
        count++;
    }
    
    return count > during;
}

bool joint_safety_check(std::vector<float> &joint_pos, std::vector<float> &joint_vel)//关节速度和位置安全检查
{
    // robot_control_param param;
    
    // std::vector<std::string> joint_types = {"hipx", "hipy", "knee"};
    
    // for (size_t i = 0; i < joint_pos.size(); i++) {
    //     std::string joint_type = joint_types[i % 3];

    //     auto it = param.joint_limit.find(joint_type);
    //     if (it == param.joint_limit.end()) {
    //         std::cerr << "Error: Joint type " << joint_type << " not found in joint_limit map" << std::endl;
    //         return false;
    //     }
        
    //     const std::vector<float>& limits = it->second;
        
    //     if (joint_pos[i] < limits[0] || joint_pos[i] > limits[1]) {
    //         std::cerr << "Warning: Joint " << i << " (" << param.joint_name[i] 
    //                   << ") position out of range: " << joint_pos[i] 
    //                   << " (limit: [" << limits[0] 
    //                   << ", " << limits[1] << "])" << std::endl;
    //         return false;
    //     }
    // }
    
    // for (size_t i = 0; i < joint_vel.size(); i++) {
    //     int joint_type = i % 4;
    //     float max_speed = (joint_type == 3) ? param.Max_speed_wheel : param.Max_speed_joint;
        
    //     if (std::abs(joint_vel[i]) > max_speed) {
    //         std::cerr << "Warning: Joint " << i << " (" << param.joint_name[i] 
    //                   << ") velocity too high: " << joint_vel[i] 
    //                   << " rad/s (max: " << max_speed << " rad/s)" << std::endl;
    //         return false;
    //     }
    // }
    
    return true;
}

std::vector<float> QuatRotateInverse(const std::vector<float>& q, const std::vector<float>& v)
{
    float q_w = q[0];
    float q_x = q[1];
    float q_y = q[2];
    float q_z = q[3];

    float v_x = v[0];
    float v_y = v[1];
    float v_z = v[2];

    float a_x = v_x * (2.0f * q_w * q_w - 1.0f);
    float a_y = v_y * (2.0f * q_w * q_w - 1.0f);
    float a_z = v_z * (2.0f * q_w * q_w - 1.0f);

    float cross_x = q_y * v_z - q_z * v_y;
    float cross_y = q_z * v_x - q_x * v_z;
    float cross_z = q_x * v_y - q_y * v_x;

    float b_x = cross_x * q_w * 2.0f;
    float b_y = cross_y * q_w * 2.0f;
    float b_z = cross_z * q_w * 2.0f;

    float dot = q_x * v_x + q_y * v_y + q_z * v_z;

    float c_x = q_x * dot * 2.0f;
    float c_y = q_y * dot * 2.0f;
    float c_z = q_z * dot * 2.0f;

    return {a_x - b_x + c_x, a_y - b_y + c_y, a_z - b_z + c_z};
}

void data_clip(std::vector<float> &data, int clip_num)
{
    float scale_factor = std::pow(10.0f, static_cast<float>(clip_num));
    
    for (auto &value : data)
    {
        value = std::round(value * scale_factor);
        value = value / scale_factor;
    }
}

int getCycleCount( float second,float frequency)
{
    return static_cast<int>(second * frequency);
}