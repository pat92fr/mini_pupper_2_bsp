
#ifndef _ML_API_H
#define _ML_API_H

#include "Eigen"

namespace mini_pupper
{

    struct ml_api
    {
        void set_position(Eigen::Matrix<float,3,4> const & joint_position_rad) const
        {
            // position = (joint_position_rad - calibration_position_rad) * direction * scale + 512
            Eigen::Matrix<float,3,4> a = joint_position_rad-calibration_position_rad;
            Eigen::Matrix<float,3,4> b = a.cwiseProduct(direction);
            Eigen::Matrix<float,3,4> servo_position = (b*scale).array()+512.0f;
            std::cout<< servo_position << std::endl;
        }

        Eigen::Matrix<float,3,4> calibration_position_rad {
            { 0.0f,     0.0f,      0.0f,     0.0f  },
            { 2.47f,    2.47f,     2.47f,    2.47f },
            { 0.86f,    0.86f,     0.86f,    0.86f }
        };

        Eigen::Matrix<float,3,4> direction {
            { 1.0f,     1.0f,     -1.0f,     -1.0f },
            { 1.0f,    -1.0f,      1.0f,     -1.0f },
            { 1.0f,    -1.0f,      1.0f,     -1.0f }
        };

        float scale { 1024.0f/5.236f };
    };

};

#endif //_ML_API_H
