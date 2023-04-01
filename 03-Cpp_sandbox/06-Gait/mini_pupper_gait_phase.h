#ifndef MINI_PUPPER_GAIT_PHASE_H_INCLUDED
#define MINI_PUPPER_GAIT_PHASE_H_INCLUDED

namespace mini_pupper
{
    // Phase :
    //  convert real-time into state and alpha [0,1[ for one leg
    //  when not walking, state = STANCE and alpha = 0.5

    struct gait_phase
    {
        gait_phase(
            config & cfg,
            LEG_ID leg
        ) :
        _cfg(cfg),
        _leg(leg)
        {};

        void update(
            float time_s,
            bool walk = false
        )
        {
            // compute time elapsed in the present cycle according leg phase
            float local_time_s { fmod(time_s-_cfg.stance_offset[_leg]*_cfg.period_s+0.5f*_cfg.stance_s,_cfg.period_s) };

            // walking state machine
            if(_walking)
            {
                if( !walk && (_last_local_time_s<(0.5f*_cfg.stance_s)) && (local_time_s>=(0.5f*_cfg.stance_s)) )
                    _walking = false;
            }
            else
            {
                if( walk && (_last_local_time_s<(0.5f*_cfg.stance_s)) && (local_time_s>=(0.5f*_cfg.stance_s)) )
                    _walking = true;
            }
            // save time for walking state machine transition
            _last_local_time_s = local_time_s;

            // compute state and alpha according time and config
            if(_walking)
            {
                if(local_time_s<_cfg.stance_s)
                {
                    _state = PHASE_STANCE;
                    _alpha = local_time_s/_cfg.stance_s;
                }
                else
                {
                    _state = PHASE_SWING;
                    _alpha = (local_time_s-_cfg.stance_s)/(_cfg.period_s-_cfg.stance_s);
                }
            }
            else // standby
            {
                _state = PHASE_STANCE;
                _alpha = 0.5f;
            }
        }

        PHASE_STATE get_state() const
        {
            return _state;
        }

        float get_alpha() const
        {
            return _alpha;
        }

        bool is_walking() const
        {
            return _walking;
        }

    private:

        // reference to current configuration
        config & _cfg;

        // this leg
        LEG_ID _leg;

        // state : STANCE or SWING
        PHASE_STATE _state {PHASE_STANCE};

        // alpha : [0,1[
        float _alpha {0};

        // leg is walking or not
        bool _walking {false};

        // last iteration local real-time
        float _last_local_time_s {0};
    };

};

#endif // MINI_PUPPER_GAIT_PHASE_H_INCLUDED
