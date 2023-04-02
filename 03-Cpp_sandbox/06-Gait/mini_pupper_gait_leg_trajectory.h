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
        _phase(phase)
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

        }

    private:

        // reference to current configuration
        config & _cfg;

        // this leg
        LEG_ID _leg;

        // reference to source phase
        gait_phase & _phase;

    };


};

#endif // MINI_PUPPER_GAIT_LEG_TRAJECTORY_H_INCLUDED
