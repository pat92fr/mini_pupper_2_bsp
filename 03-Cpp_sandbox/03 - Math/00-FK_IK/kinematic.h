
#ifndef KINEMATIC_H
#define KINEMATIC_H

#include "Eigen"
#include <cmath>
#include <algorithm>

namespace mini_pupper
{
    //inline float to_radians(float value = 1.0) { return value * M_PI/180.0f; };
    //inline float to_degrees(float value = 1.0) { return value * 180.0f/M_PI; };
    
    template<typename T>
    T to_radians(T const & value) { return value * (M_PI/180.0f); };
    template<typename T>
    T to_degrees(T const & value) { return value * (180.0f/M_PI); };
    

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

        Eigen::Matrix<float,3,4> four_leg_forward_kinematics_LRF(Eigen::Matrix<float,3,4> const & joint_position) const
        {
            Eigen::Matrix<float,3,4> m;
            m.col(LEG_FR) = leg_forward_kinematics_LRF(joint_position.col(LEG_FR),LEG_FR);
            m.col(LEG_FL) = leg_forward_kinematics_LRF(joint_position.col(LEG_FL),LEG_FL);
            m.col(LEG_RR) = leg_forward_kinematics_LRF(joint_position.col(LEG_RR),LEG_RR);
            m.col(LEG_RL) = leg_forward_kinematics_LRF(joint_position.col(LEG_RL),LEG_RL);
            return m;
        }

        Eigen::Vector3f leg_inverse_kinematics_LRF(Eigen::Vector3f const & foot_position, LEG_ID leg_index) const
        {
            float const CF_dist_sq { powf(foot_position[0],2.0f) + powf(foot_position[1],2.0f) };
            float const CF_dist { sqrtf(CF_dist_sq) };
            float const CF_angle { atan2f(-foot_position[1],-foot_position[0]) };
            float const HF_dist_sq { max( CF_dist_sq - powf(ABDUCTION_OFFSET,2.0f) , 0.0f ) };
            float const HF_dist { sqrtf( HF_dist_sq ) };
            float const FCH { cos_law_A(CF_dist,ABDUCTION_OFFSET,HF_dist) };
            static float const ANGLE_SIGN[4] { 1.0f, -1.0f, 1.0f, -1.0f };
            float const HIPS_A { CF_angle + ANGLE_SIGN[leg_index] * (FCH - M_PI/2.0f) };
            float const L { sqrtf( powf(foot_position[2],2.0f) + HF_dist_sq) };
            float const Alpha { atan2f(HF_dist,-foot_position[2]) };
            float const HIPS { Alpha + cos_law_A(L,LEG_LF,LEG_LT) };
            float const KNEE { cos_law_A(LEG_LF,LEG_LT,L) - M_PI + HIPS };
            /*
            std::cout << "CF_dist: " << CF_dist << std::endl;
            std::cout << "CF_angle: " << to_degrees(CF_angle) << std::endl;
            std::cout << "HF_dist: " << HF_dist << std::endl;
            std::cout << "FCH: " << to_degrees(FCH) << std::endl;
            */
            return Eigen::Vector3f {HIPS_A,HIPS,KNEE};
            // todo:    joint_position[1:2] = np.fmod(joint_position[1:2] + 2*np.pi, 2 * np.pi)
        }

        Eigen::Matrix<float,3,4> four_leg_inverse_kinematics_LRF(Eigen::Matrix<float,3,4> const & foot_position) const
        {
            Eigen::Matrix<float,3,4> m;
            m.col(LEG_FR) = leg_inverse_kinematics_LRF(foot_position.col(LEG_FR),LEG_FR);
            m.col(LEG_FL) = leg_inverse_kinematics_LRF(foot_position.col(LEG_FL),LEG_FL);
            m.col(LEG_RR) = leg_inverse_kinematics_LRF(foot_position.col(LEG_RR),LEG_RR);
            m.col(LEG_RL) = leg_inverse_kinematics_LRF(foot_position.col(LEG_RL),LEG_RL);
            return m;
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

        inline float cos_law_c(float a, float b, float angle) const
        {
            return sqrtf(a*a+b*b-2.0f*a*b*cosf(angle));
        };

        inline float cos_law_A(float a, float b, float c) const
        {
            float const tmp { (a*a+b*b-c*c)/(2.0f*a*b) };
            return acosf( min( max( tmp, -1.0f ), 1.0f) );
        };

    };

}

#endif //KINEMATIC_H
