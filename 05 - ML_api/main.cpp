#include <iostream>

#include "ml-api.h"
using namespace mini_pupper;

ml_api ml;

int main(int argc, char *argv[])
{
    std::cout << "Hello world!" << std::endl;

    Eigen::Matrix<float,3,4>joint_position_rad {
        {0.0f, 0.0f, 0.0f },
        {0.0f, 0.0f, 0.0f },
        {0.0f, 0.0f, 0.0f }
    };
    ml.set_position(joint_position_rad);



    return 0;
}
