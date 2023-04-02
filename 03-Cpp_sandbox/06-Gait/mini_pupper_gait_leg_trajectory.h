#ifndef MINI_PUPPER_GAIT_LEG_TRAJECTORY_H_INCLUDED
#define MINI_PUPPER_GAIT_LEG_TRAJECTORY_H_INCLUDED

#include "Eigen"
#include <cmath>
#include <algorithm>

namespace mini_pupper
{

    struct leg_trajectory
    {
        leg_trajectory(
            config & cfg,
            LEG_ID leg,
            gait_phase & phase
        ) :
        _cfg(cfg),
        _leg(leg),
        _phase(phase),
        _coc(_cfg)
        {};

        void update(
            float vx,
            float vy,
            float wz,
            float dx,
            float dy,
            float dz,
            float pitch,
            float roll,
            float yaw
        )
        {
            // 1. capture speed when leg centered
            if(_phase.is_centered())
            {
                _vx = vx;
                _vy = vy;
                _wz = wz;
                _coc.update(_vx,_vy,_wz);
            }

            // turning





        }

    private:

        // reference to current configuration
        config & _cfg;

        // this leg
        LEG_ID _leg;

        // reference to source phase
        gait_phase & _phase;

        // internal velocity
        float _vx {0.0f};
        float _vy {0.0f};
        float _wz {0.0f};

        // internal coc
        center_of_curvature _coc;

    };


};

#endif // MINI_PUPPER_GAIT_LEG_TRAJECTORY_H_INCLUDED
