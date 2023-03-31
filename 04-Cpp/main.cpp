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
    
    if(false)
    {
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

        // free joints
        for(size_t index=0; index<12; ++index)
        {
            control.goal_position[index]= 512;
            control.torque_enable[index] = 0;
        }
        int result = servo.update(control,feedback);
        std::cout << "result:" << result << std::endl;

        // speed test
        milliseconds t0_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
        int errors {0};
        for(int i=0;i<1000;++i)
        {
            //Eigen::Matrix<float,3,4> pose_BRF = kin.crouch_pose_BRF;
            Eigen::Matrix<float,3,4> pose_BRF = kin.standby_pose_BRF;

            //std::cout << "pose_BRF:\n" << standby_pose_BRF << std::endl;

            Eigen::Matrix<float,3,4> const joint_angle { kin.four_leg_inverse_kinematics_BRF(pose_BRF) };

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

        // free joints
        for(size_t index=0; index<12; ++index)
        {
            control.goal_position[index]= 512;
            control.torque_enable[index] = 0;
        }
        result = servo.update(control,feedback);
        std::cout << "result:" << result << std::endl;
    }

    // test 2 : circle BW
    std::cout << "test #2" << std::endl;
    if(true)
    {
        float const duration_s {5.000}; 
        float const radius_m {0.020}; 
        float const period_s {0.200};

        milliseconds t0_ms { duration_cast< milliseconds >(system_clock::now().time_since_epoch()) };
        int errors {0};
        bool running {true};
        do
        {
            milliseconds const t_ms {duration_cast< milliseconds >(system_clock::now().time_since_epoch())};
            duration<float> const difference { t_ms - t0_ms };
            float const ellapsed_time_s {difference.count()};
            //std::cout << ellapsed_time_s << std::endl;

            float const dx { radius_m * cosf( ellapsed_time_s / period_s * 2.0f * (float)M_PI) };
            float const dz { radius_m * sinf( ellapsed_time_s / period_s * 2.0f * (float)M_PI) };
            //std::cout << "(" << dx << "," << dz << ")" << std::endl;

            Eigen::Matrix<float,3,4> const delta_pose_BRF {
                { dx,   dx,   dx,   dx   },
                { 0.0f, 0.0f, 0.0f, 0.0f },
                { dz,   dz,   dz,   dz   },
            };
            
            Eigen::Matrix<float,3,4> const pose_BRF { kin.standby_pose_BRF+delta_pose_BRF };

            Eigen::Matrix<float,3,4> const joint_angle { kin.four_leg_inverse_kinematics_BRF(pose_BRF) };

            int position_setpoint[12] {0};
            jo.position_setpoint(joint_angle,position_setpoint);
            
            for(size_t index=0; index<12; ++index)
            {
                control.goal_position[index]= position_setpoint[index];
                control.torque_enable[index] = 1;
            }

            int result { servo.update(control,feedback) };
            if(result!=mini_pupper::API_OK) ++errors;


            running = difference.count()<duration_s;
        } while(running);
        std::cout << "circle BW: (erros count:" << errors << ")" << std::endl;

        // free joints
        for(size_t index=0; index<12; ++index)
        {
            control.goal_position[index]= 512;
            control.torque_enable[index] = 0;
        }
        int result = servo.update(control,feedback);
        std::cout << "result:" << result << std::endl;

    }
    std::cout << "test 2 finished." << std::endl;

    return 0;
}
