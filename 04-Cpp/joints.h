
#ifndef _JOINTS_H
#define _JOINTS_H

#include "Eigen"
#include "Dense"

namespace mini_pupper
{

    struct ml_api
    {
        void set_position(Eigen::Array<float,3,4> const & joint_position_rad) const
        {
            Eigen::Array<float,3,4> servo_position_f = (joint_position_rad-calibration_position_rad)*direction*scale+512.0f;
            Eigen::Array<int,3,4> servo_position = servo_position_f.cast<int>();

            //std::cout  << servo_position << servo_position.reshaped(12,1) << std::endl;
            int * servo = servo_position.data();
            for(size_t index=0; index<12; ++index)
            {
                std::cout  << servo[index] << std::endl;
            }

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

#endif //_JOINTS_H
