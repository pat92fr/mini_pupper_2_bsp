#ifndef MINI_PUPPER_CONFIG_H_INCLUDED
#define MINI_PUPPER_CONFIG_H_INCLUDED

namespace mini_pupper
{

    struct config
    {
        // gait config
        float period_s {0.500f};
        float stance_s {0.300f};
        float stance_offset[4] { 0.0f, 0.5f, 0.0f, 0.5f };


    };

};

#endif // MINI_PUPPER_CONFIG_H_INCLUDED
