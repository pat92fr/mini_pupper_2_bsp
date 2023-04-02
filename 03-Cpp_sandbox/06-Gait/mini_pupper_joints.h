#ifndef MINI_PUPPER_JOINTS_H_INCLUDED
#define MINI_PUPPER_JOINTS_H_INCLUDED


#include "Eigen"
#include "Dense"


#include <algorithm>

namespace mini_pupper
{

    struct joints
    {
        void position_setpoint(Eigen::Array<float,3,4> const & joint_position_rad, int servo_position[12]) const
        {
            Eigen::Array<float,3,4> servo_position_f = (joint_position_rad-calibration_position_rad)*direction*scale+512.0f;
            Eigen::Array<int,3,4> servo_position_i = servo_position_f.cast<int>();
            std::copy(servo_position_i.data(), servo_position_i.data()+12, servo_position);
        }

        Eigen::Array<float,3,4> calibration_position_rad {
            { 0.0f,     0.0f,      0.0f,     0.0f  },
            { 2.47f,    2.47f,     2.47f,    2.47f },
            { 0.86f,    0.86f,     0.86f,    0.86f }
        };

        Eigen::Array<float,3,4> direction {
            { 1.0f,     1.0f,     -1.0f,     -1.0f },
            { 1.0f,    -1.0f,      1.0f,     -1.0f },
            { 1.0f,    -1.0f,      1.0f,     -1.0f }
        };

        Eigen::Array<float,3,4> scale {
            { 1024.0f/5.236f,    1024.0f/5.236f,     1024.0f/5.236f,     1024.0f/5.236f },
            { 1024.0f/5.236f,    1024.0f/5.236f,     1024.0f/5.236f,     1024.0f/5.236f },
            { 1024.0f/5.236f,    1024.0f/5.236f,     1024.0f/5.236f,     1024.0f/5.236f }
        };

    };

};

#endif // MINI_PUPPER_JOINTS_H_INCLUDED
