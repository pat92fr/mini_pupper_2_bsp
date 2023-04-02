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
            config & cfg,
            LEG_ID leg,
            kinematic & kin
        ) :
        _cfg(cfg),
        _leg(leg),
        _foot_origin_BRF(kin.STANDBY_POSE_BRF.col(_leg))
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
                if(wz>=0.0f)
                    angle = atan2f(vy,vx)+0.5f*M_PI;
                else
                    angle = atan2f(vy,vx)-0.5f*M_PI;
                position_BRF[0] = distance*cosf(angle);
                position_BRF[1] = distance*sinf(angle);
                position_BRF[2] = _foot_origin_BRF[2];
                Eigen::Vector3f const coc_to_foot {_foot_origin_BRF-position_BRF};
                foot_distance = coc_to_foot.norm();
                foot_angle = atan2f(coc_to_foot[1],coc_to_foot[0]);
            }
            else
            {
                distance = 0.0f;
                angle = 0.0f;
                position_BRF = Eigen::Vector3f::Zero();
                foot_distance = 0.0f;
                foot_angle = 0.0f;
            }
        }

        // distance from CoM in BRF
        float distance {0.0f};

        // CoC angle (TRIGO) about Z axis in BRF
        float angle {0.0f};

        // CoC position in BRF
        Eigen::Vector3f position_BRF {0.0f, 0.0f, 0.0f };

        // CoC to foot distance
        float foot_distance {0.0f};

        // CoC to foot angle
        float foot_angle {0.0f};

        // turning
        bool turning {false};

    private:

        // reference to current configuration
        config & _cfg;

        // this leg
        LEG_ID _leg;

        // position of foot in BRF
        Eigen::Vector3f _foot_origin_BRF {0.0f, 0.0f, 0.0f };

    };

};

#endif // MINI_PUPPER_GAIT_CENTER_OF_CURVATURE_H_INCLUDED
