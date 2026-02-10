#include <csignal>
#include <iostream>
#include <ros/ros.h>
#include "robot_param.hpp"
#include "FSM/FSM_Manager.hpp"
#include "Timer/thread_Timer.hpp"

class deploy
{
private:
    FSM_Manager FSM_Deploy;
    thread_Timer rl_Deploy_thread;
    robot_control_param param;

public:
    deploy(const std::string &sim_engine, const std::string &model_type)
      :FSM_Deploy(sim_engine,model_type){
        rl_Deploy_thread.start(param.control_rate,std::bind(&FSM_Manager::run,&FSM_Deploy));
    }
  
    ~deploy(){
        rl_Deploy_thread.stop();
    }
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "go2_deploy");

    std::string sim_engine="gazebo";
    std::string model_type="torchscript";

    if (argc>3){
        ROS_ERROR("too much params");
        return 1;
    }
    else {
        for (size_t i = 1; i < argc; i++){
            std::string param=argv[i];
            if (param=="gazebo"||param=="mujoco"){
                sim_engine = param;
            }
            if (param=="torchscript"||param=="onnx"){
                model_type = param;
            }
        } 
    }

    deploy Robot_Deploy(sim_engine,model_type);

    ros::spin();   
    return 0;
}