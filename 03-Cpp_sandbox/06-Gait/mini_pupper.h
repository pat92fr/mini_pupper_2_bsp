#ifndef MINI_PUPPER_H_INCLUDED
#define MINI_PUPPER_H_INCLUDED

namespace mini_pupper
{

    enum LEG_ID
    {
        LEG_FR = 0,
        LEG_FL = 1,
        LEG_RR = 2,
        LEG_RL = 3
    };

    enum PHASE_STATE
    {
        PHASE_STANCE = 0,
        PHASE_SWING
    };

};

#include "mini_pupper_config.h"
#include "mini_pupper_kinematics.h"
#include "mini_pupper_gait_phase.h"
#include "mini_pupper_gait_velocity_smoother.h"
#include "mini_pupper_gait_center_of_curvature.h"

#endif // MINI_PUPPER_H_INCLUDED
