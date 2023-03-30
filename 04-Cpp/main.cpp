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

    float const STANCE_X {0.059f};
    float const STANCE_Y {0.050f};
    float const STANCE_Z {-0.080f};
    float const STANCE_X_SHIFT {0.000f};

    Eigen::Matrix<float,3,4> const standby_pose_BRF {
        {  STANCE_X+STANCE_X_SHIFT, STANCE_X+STANCE_X_SHIFT,-STANCE_X+STANCE_X_SHIFT,  -STANCE_X+STANCE_X_SHIFT },
        { -STANCE_Y,                STANCE_Y,               -STANCE_Y,                  STANCE_Y },
        {  STANCE_Z,                STANCE_Z,                STANCE_Z,                  STANCE_Z }
    };
    
    // test 1
    std::cout << "test #1" << std::endl;
    {
        std::cout << "standby_pose_BRF:\n" << standby_pose_BRF << std::endl;

        Eigen::Matrix<float,3,4> const joint_angle { kin.four_leg_inverse_kinematics_BRF(standby_pose_BRF) };

        std::cout << "joint_angle:\n" << joint_angle*(180.0f/M_PI) << std::endl;



    }

    milliseconds t0_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    int errors {0};
    for(int i=0;i<1000;++i)
    {
        /*
        Eigen::Matrix<float,3,4> const jp_deg {
            { 0.0f,      0.0f,      23.0f,     -15.0f },
            { 180.0f,    90.0f,     125.0f,    180.0f },
            { 90.0f,     90.0f,     45.0f,     170.0f }
        };
        Eigen::Matrix<float,3,4> const jp { mini_pupper::to_radians(jp_deg) };
        Eigen::Matrix<float,3,4> const lp { kin.four_leg_forward_kinematics_LRF(jp) };
        Eigen::Matrix<float,3,4> const jpik { kin.four_leg_inverse_kinematics_LRF(lp) };
        if(!jpik.isApprox(jp))
            ++errors;
            */
    }

    milliseconds t1_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    duration<float> difference = t1_ms - t0_ms;
    int const milliseconds = difference.count() * 1000;

    //std::cout << "1000x fk-ik delay:" << milliseconds << "ms (erros count:" << errors << ")" << std::endl;
    //std::cout << "fk-ik frequency:" << (1000000.0/milliseconds) << " Hz" << std::endl;

    return 0;
}
