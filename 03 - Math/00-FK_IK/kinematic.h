
#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <eigen>
#include <cmath>

namespace mini_pupper
{
    inline float to_radians(float value) { return value * M_PI/180.0f; };
    inline float to_degrees(float value) { return value * 180.0f/M_PI; };

    enum LEG_ID
    {
        LEG_FR = 0,
        LEG_FL = 1,
        LEG_RR = 2,
        LEG_RL = 3
    };

    struct kinematic
    {
        Eigen::Vector3f leg_forward_kinematics_LRF(Eigen::Vector3f const & joint_position, LEG_ID leg_index) const
        {
            float const c1 {  cosf(joint_position[0]) };
            float const s1 {  sinf(joint_position[0]) };
            float const c2 { -sinf(joint_position[1]) };
            float const s2 {  cosf(joint_position[1]) };
            float const c3 { -sinf(joint_position[2]) };
            float const s3 {  cosf(joint_position[2]) };

            return Eigen::Vector3f {
                c1*c3*LEG_LT + c1*c2*LEG_LF - s1*ABDUCTION_OFFSETS[leg_index],
                s1*c3*LEG_LT + s1*c2*LEG_LF + c1*ABDUCTION_OFFSETS[leg_index],
                  -s3*LEG_LT -    s2*LEG_LF,
            };
        }

        float LEG_OX {0.059f};
        float LEG_OY {0.0235f};
        float LEG_OZ {0.000f};
        float ABDUCTION_OFFSET {0.026f};
        float LEG_LF {0.050f};
        float LEG_LT {0.060f};

    private:

        float ABDUCTION_OFFSETS[4] {
            -ABDUCTION_OFFSET,
             ABDUCTION_OFFSET,
            -ABDUCTION_OFFSET,
             ABDUCTION_OFFSET,
        };
    };

}

#endif //KINEMATIC_H
