#ifndef MINI_PUPPER_GAIT_CENTER_OF_CURVATURE_H_INCLUDED
#define MINI_PUPPER_GAIT_CENTER_OF_CURVATURE_H_INCLUDED

#include "Eigen"
#include <cmath>
#include <algorithm>

namespace mini_pupper
{


    struct center_of_curvature
    {
        center_of_curvature(
            config & cfg
        ) :
        _cfg(cfg)
        {};

        void update(
            float vx,
            float vy,
            float wz )
        {
            turning = fabs(wz) >= _cfg.wz_min_rps;
            if(turning)
            {
                distance = fabs(sqrtf(vx*vx+vy*vy)/wz);
                if(wz>0.0f)
                    angle = atan2f(vy,vx)+0.5f*M_PI;
                else
                    angle = atan2f(vy,vx)-0.5f*M_PI;
                position_BRF[0] = distance*cosf(angle);
                position_BRF[1] = distance*sinf(angle);
                position_BRF[2] = 0.0f;
            }
            else
            {
                distance = 0.0f;
                angle = 0.0f;
                position_BRF = Eigen::Vector3f::Zero();
            }

        }

        // distance from CoM in BRF
        float distance {0.0f};

        // angle (TRIGO) about Z axis in BRF
        float angle {0.0f};

        // position of CoM in BRF
        Eigen::Vector3f position_BRF {0.0f, 0.0f, 0.0f };

        // turning
        bool turning {false};

    private:

        // reference to current configuration
        config & _cfg;
    };

};

#endif // MINI_PUPPER_GAIT_CENTER_OF_CURVATURE_H_INCLUDED
