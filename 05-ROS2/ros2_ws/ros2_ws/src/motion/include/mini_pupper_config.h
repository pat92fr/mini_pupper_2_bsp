#ifndef MINI_PUPPER_CONFIG_H_INCLUDED
#define MINI_PUPPER_CONFIG_H_INCLUDED

namespace mini_pupper
{

    struct config
    {
        // refresh rate
        float dt {1.0f/333.3f};

        // move config
        float vx_min_mps = 0.010f;
        float vx_max_mps = 0.300f;
        float vy_min_mps = 0.010f;
        float vy_max_mps = 0.100f;
        float wz_min_rps = 0.100f;
        float wz_max_rps = 2.000f;
        float axy_mps = 0.200f;
        float az_rps = 2.000f;

        // gait config
        float period_s {0.330f};
        float stance_s {0.180f};
        float stance_offset[4] { 0.0f, 0.5f, 0.5f, 0.0f };
        float swing_height {0.015f};


    };

};

#endif // MINI_PUPPER_CONFIG_H_INCLUDED
