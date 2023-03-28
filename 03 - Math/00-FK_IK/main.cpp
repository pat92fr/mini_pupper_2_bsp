#include <iostream>
using namespace std;

#include "kinematic.h"

int main()
{
    cout << "Hello world!" << endl;

    mini_pupper::kinematic kin;

    Eigen::Vector3f jp {
        mini_pupper::to_radians(0.0f),
        mini_pupper::to_radians(180.0f),
        mini_pupper::to_radians(90.0f)
    };
    Eigen::Vector3f lp { kin.leg_forward_kinematics_LRF(jp,mini_pupper::LEG_FR) };

    std::cout << lp << std::endl;

    return 0;
}
