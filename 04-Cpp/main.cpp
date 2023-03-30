#include <iostream>
using namespace std;

#include <chrono>
using namespace std::chrono;

#include "kinematic.h"
#include "joints.h"
#include "esp32-api.h"

int main()
{
    cout << "Hello world!" << endl;

    mini_pupper::kinematic kin;
    mini_pupper::joints jo;

    mini_pupper::esp32_api servo;
    mini_pupper::parameters_control_instruction_format control;
    mini_pupper::parameters_control_acknowledge_format feedback;
    
    // test 1
    std::cout << "test #1" << std::endl;
    {
        Eigen::Matrix<float,3,4> standby_pose_BRF = kin.standby_pose_BRF;

        std::cout << "standby_pose_BRF:\n" << standby_pose_BRF << std::endl;

        Eigen::Matrix<float,3,4> const joint_angle { kin.four_leg_inverse_kinematics_BRF(standby_pose_BRF) };

        std::cout << "joint_angle:\n" << joint_angle*(180.0f/M_PI) << std::endl;

        int position_setpoint[12] {0};

        jo.position_setpoint(joint_angle,position_setpoint);
        
        std::cout << "position_setpoint:\n";
        for(auto & p : position_setpoint) std::cout << p << " ";
        std::cout << std::endl;

        for(size_t index=0; index<12; ++index)
        {
            control.goal_position[index]= position_setpoint[index];
            control.torque_enable[index] = 1;
        }

        int result = servo.update(control,feedback);
        std::cout << "result:" << result << std::endl;

        //Eigen::Matrix<float,3,4> const lp { kin.four_leg_forward_kinematics_LRF(jp) };
    }

    for(size_t index=0; index<12; ++index)
    {
        control.goal_position[index]= 512;
        control.torque_enable[index] = 0;
    }

    int result = servo.update(control,feedback);
    std::cout << "result:" << result << std::endl;

    milliseconds t0_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    int errors {0};
    for(int i=0;i<1000;++i)
    {
        Eigen::Matrix<float,3,4> standby_pose_BRF = kin.standby_pose_BRF;

        //std::cout << "standby_pose_BRF:\n" << standby_pose_BRF << std::endl;

        Eigen::Matrix<float,3,4> const joint_angle { kin.four_leg_inverse_kinematics_BRF(standby_pose_BRF) };

        //std::cout << "joint_angle:\n" << joint_angle*(180.0f/M_PI) << std::endl;

        int position_setpoint[12] {0};

        jo.position_setpoint(joint_angle,position_setpoint);
        
        //std::cout << "position_setpoint:\n";
        //for(auto & p : position_setpoint) std::cout << p << " ";
        //std::cout << std::endl;

        for(size_t index=0; index<12; ++index)
        {
            control.goal_position[index]= position_setpoint[index];
            control.torque_enable[index] = 1;
        }

        int result = servo.update(control,feedback);
        if(result!=mini_pupper::API_OK) ++errors;

        //Eigen::Matrix<float,3,4> const lp { kin.four_leg_forward_kinematics_LRF(jp) };
    }

    milliseconds t1_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    duration<float> difference = t1_ms - t0_ms;
    int const milliseconds = difference.count() * 1000;

    std::cout << "1000x ik-bus delay:" << milliseconds << "ms (erros count:" << errors << ")" << std::endl;
    std::cout << "ik-bus frequency:" << (1000000.0/milliseconds) << " Hz" << std::endl;

    for(size_t index=0; index<12; ++index)
    {
        control.goal_position[index]= 512;
        control.torque_enable[index] = 0;
    }

    result = servo.update(control,feedback);
    std::cout << "result:" << result << std::endl;

    return 0;
}
