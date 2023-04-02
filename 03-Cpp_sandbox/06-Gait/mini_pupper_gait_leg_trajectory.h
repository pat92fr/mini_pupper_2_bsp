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
            gait_phase & phase,
            kinematic & kin
        ) :
        _cfg(cfg),
        _leg(leg),
        _phase(phase),
        _coc(_cfg,_leg,kin),
        _foot_origin_BRF(kin.STANDBY_POSE_BRF.col(_leg))
        {};

        void update(
            float vx,
            float vy,
            float wz
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
            if(_coc.turning)
            {
                if(_phase.get_state()==PHASE_STANCE)
                {
                    Eigen::Vector3f const delta {
                        _coc.foot_distance*cosf(_coc.foot_angle-wz*_cfg.stance_s*(_phase.get_alpha()-0.5f)),
                        _coc.foot_distance*sinf(_coc.foot_angle-wz*_cfg.stance_s*(_phase.get_alpha()-0.5f)),
                        0.0f
                    };
                    _foot_BRF = _coc.position_BRF + delta;
                }
                else // PHASE_SWING
                {

                    // TODO LINEAR
                    // TODO LINEAR
                    // TODO LINEAR
                    _foot_BRF = _foot_origin_BRF;
                }
            }
            else // straight
            {
                if(_phase.get_state()==PHASE_STANCE)
                {
                    Eigen::Vector3f const delta {
                        -vx*_cfg.stance_s*(_phase.get_alpha()-0.5f),
                        -vy*_cfg.stance_s*(_phase.get_alpha()-0.5f),
                        0.0f
                    };
                    _foot_BRF = _foot_origin_BRF + delta;
                }
                else // PHASE_SWING
                {
                    Eigen::Vector3f const delta {
                        vx*(_cfg.period_s-_cfg.stance_s)*(_phase.get_alpha()-0.5f),
                        vy*(_cfg.period_s-_cfg.stance_s)*(_phase.get_alpha()-0.5f),
                        _cfg.swing_height*sinf(_phase.get_alpha()*M_PI)
                    };
                    _foot_BRF = _foot_origin_BRF + delta;
                }
            }
        }

        Eigen::Vector3f const & get_foot_BRF() const
        {
            return _foot_BRF;
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

        // position of foot in BRF
        Eigen::Vector3f _foot_origin_BRF {0.0f, 0.0f, 0.0f };

        // resulting position of foot in BRF
        Eigen::Vector3f _foot_BRF {0.0f, 0.0f, 0.0f };

    };


};

#endif // MINI_PUPPER_GAIT_LEG_TRAJECTORY_H_INCLUDED
