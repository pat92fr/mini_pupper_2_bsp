#include <iostream>
using namespace std;

#include <chrono>
using namespace std::chrono;

#include "kinematic.h"

int main()
{
    cout << "Hello world!" << endl;

    mini_pupper::kinematic kin;

    // test 1
    std::cout << "test #1" << std::endl;
    {
        Eigen::Vector3f jp {
            mini_pupper::to_radians(0.0f),
            mini_pupper::to_radians(180.0f),
            mini_pupper::to_radians(90.0f)
        };

        std::cout << jp << std::endl;

        Eigen::Vector3f lp { kin.leg_forward_kinematics_LRF(jp,mini_pupper::LEG_FR) };

        std::cout << lp << std::endl;

        Eigen::Vector3f jpik { kin.leg_inverse_kinematics_LRF(lp,mini_pupper::LEG_FR) };

        std::cout << jpik << std::endl;
    }

    // test 2
    std::cout << "test #2" << std::endl;
    {
        Eigen::Matrix<float,3,4> jp {
            { mini_pupper::to_radians(0.0f),      mini_pupper::to_radians(0.0f),      mini_pupper::to_radians(23.0f),     mini_pupper::to_radians(-15.0f) },
            { mini_pupper::to_radians(180.0f),    mini_pupper::to_radians(90.0f),     mini_pupper::to_radians(125.0f),    mini_pupper::to_radians(180.0f) },
            { mini_pupper::to_radians(90.0f),     mini_pupper::to_radians(90.0f),     mini_pupper::to_radians(45.0f),     mini_pupper::to_radians(170.0f) }
        };

        std::cout << jp << std::endl;

        Eigen::Matrix<float,3,4> lp { kin.four_leg_forward_kinematics_LRF(jp) };

        std::cout << lp << std::endl;

        Eigen::Matrix<float,3,4> jpik { kin.four_leg_inverse_kinematics_LRF(lp) };

        std::cout << jpik << std::endl;
    }

    milliseconds t0_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    int errors {0};
    for(int i=0;i<1000;++i)
    {
        Eigen::Matrix<float,3,4> jp {
            { mini_pupper::to_radians(0.0f),      mini_pupper::to_radians(0.0f),      mini_pupper::to_radians(23.0f),     mini_pupper::to_radians(-15.0f) },
            { mini_pupper::to_radians(180.0f),    mini_pupper::to_radians(90.0f),     mini_pupper::to_radians(125.0f),    mini_pupper::to_radians(180.0f) },
            { mini_pupper::to_radians(90.0f),     mini_pupper::to_radians(90.0f),     mini_pupper::to_radians(45.0f),     mini_pupper::to_radians(170.0f) }
        };
        Eigen::Matrix<float,3,4> lp { kin.four_leg_forward_kinematics_LRF(jp) };
        Eigen::Matrix<float,3,4> jpik { kin.four_leg_inverse_kinematics_LRF(lp) };
        if(jpik!=jp)
            ++errors;
    }

    milliseconds t1_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    duration<float> difference = t1_ms - t0_ms;
    int const milliseconds = difference.count() * 1000;

    std::cout << "1000x fk-ik delay:" << milliseconds << "ms (erros count:" << errors << ")" << std::endl;
    std::cout << "fk-ik frequency:" << (1000000.0/milliseconds) << " Hz" << std::endl;

    return 0;
}
