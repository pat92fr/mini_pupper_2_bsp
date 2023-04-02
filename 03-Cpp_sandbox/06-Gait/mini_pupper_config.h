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
        float vx_max_mps = 0.100f;
        float vy_min_mps = 0.010f;
        float vy_max_mps = 0.050f;
        float wz_min_rps = 0.100f; //  5 dps
        float wz_max_rps = 1.500f; // 90 dps
        float axy_mps = 0.100f;
        float az_rps = 1.500f;

        // gait config
        float period_s {0.500f};
        float stance_s {0.240f};
        float stance_offset[4] { 0.0f, 0.5f, 0.0f, 0.5f };
        float swing_height {0.010f};


    };

};

#endif // MINI_PUPPER_CONFIG_H_INCLUDED
