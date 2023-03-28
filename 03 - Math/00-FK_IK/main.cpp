#include <iostream>
using namespace std;

#include "kinematic.h"

int main()
{
    cout << "Hello world!" << endl;

    mini_pupper::kinematic kin;

    // test 1
    {
        Eigen::Vector3f jp {
            mini_pupper::to_radians(0.0f),
            mini_pupper::to_radians(180.0f),
            mini_pupper::to_radians(90.0f)
        };
        Eigen::Vector3f lp { kin.leg_forward_kinematics_LRF(jp,mini_pupper::LEG_FR) };

        std::cout << lp << std::endl;
    }

    // test 2
    {
        Eigen::Matrix<float,3,4> jp {
            { mini_pupper::to_radians(0.0f),      mini_pupper::to_radians(0.0f),      mini_pupper::to_radians(23.0f),     mini_pupper::to_radians(-15.0f) },
            { mini_pupper::to_radians(180.0f),    mini_pupper::to_radians(90.0f),     mini_pupper::to_radians(125.0f),    mini_pupper::to_radians(180.0f) },
            { mini_pupper::to_radians(90.0f),     mini_pupper::to_radians(90.0f),     mini_pupper::to_radians(45.0f),     mini_pupper::to_radians(170.0f) }
        };
        Eigen::Matrix<float,3,4> lp { kin.four_leg_forward_kinematics_LRF(jp) };

        std::cout << lp << std::endl;
    }

    return 0;
}
