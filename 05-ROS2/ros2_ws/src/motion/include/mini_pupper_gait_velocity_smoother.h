#ifndef MINI_PUPPER_GAIT_VELOCITY_SMOOTHER_H_INCLUDED
#define MINI_PUPPER_GAIT_VELOCITY_SMOOTHER_H_INCLUDED

namespace mini_pupper
{

    // Velocity Smoother :
    // - Convert input (manual/navigation) velocity setpoint into filtered velocity according configuration acceleration XYZ
    // - Trigger gait based on configuration minimum velocity (moving)

    struct velocity_smoother
    {
        velocity_smoother(
            config & cfg
        ) :
        _cfg(cfg)
        {};

        void update(
            float vx,
            float vy,
            float wz
        )
        {
            if(vx>_vx_filtered)
                _vx_filtered = min( _cfg.vx_max_mps, min( vx, _vx_filtered+_cfg.dt*_cfg.axy_mps ) );
            if(vx<_vx_filtered)
                _vx_filtered = max( -_cfg.vx_max_mps, max( vx, _vx_filtered-_cfg.dt*_cfg.axy_mps ) );
            if(vy>_vy_filtered)
                _vy_filtered = min( _cfg.vy_max_mps, min( vy, _vy_filtered+_cfg.dt*_cfg.axy_mps ) );
            if(vy<_vy_filtered)
                _vy_filtered = max( -_cfg.vy_max_mps, max( vy, _vy_filtered-_cfg.dt*_cfg.axy_mps ) );
            if(wz>_wz_filtered)
                _wz_filtered = min( _cfg.wz_max_rps, min( wz, _wz_filtered+_cfg.dt*_cfg.az_rps ) );
            if(wz<_wz_filtered)
                _wz_filtered = max( -_cfg.wz_max_rps, max( wz, _wz_filtered-_cfg.dt*_cfg.az_rps ) );

        }

        float get_vx() const { return _vx_filtered; }
        float get_vy() const { return _vy_filtered; }
        float get_wz() const { return _wz_filtered; }

        bool is_moving() const
        {
            return fabs(_vx_filtered) >_cfg.vx_min_mps || fabs(_vy_filtered)>_cfg.vy_min_mps || fabs(_wz_filtered)>_cfg.wz_min_rps;
        }

    private:

        // reference to current configuration
        config & _cfg;

        // internal state
        float _vx_filtered {0.0f};
        float _vy_filtered {0.0f};
        float _wz_filtered {0.0f};

    };

};

#endif // MINI_PUPPER_GAIT_VELOCITY_SMOOTHER_H_INCLUDED
