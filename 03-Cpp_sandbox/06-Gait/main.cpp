#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

#include "mini_pupper.h"
using namespace mini_pupper;

// TODO : one leg trajectory generator : configuration, filtered velocity, CoC computation (curve), phase
// TODO : config(height) + (state, alpha, vx, vy, wz filtered) ==> dX,dY,dZ around STANCE POSE
// CoM dynamic control

int main()
{
    cout << "Hello world!" << endl;

    config cfg;
    velocity_smoother vel_smo(cfg);
    vector<gait_phase> phase {
        { cfg, LEG_FR },
        { cfg, LEG_FL },
        { cfg, LEG_RR },
        { cfg, LEG_RL }
    };
    center_of_curvature coc(cfg);
    kinematic kin;

    // simulate timer-task
    ofstream file("gai.csv");
    file << "time_s" << ";" ;
    file << "vx" << ";" << "vel_smo.get_vx()" << ";";
    file << "vy" << ";" << "vel_smo.get_vy()" << ";";
    file << "wz" << ";" << "vel_smo.get_wz()" << ";";
    file << "coc.distance" << ";" << "coc.angle" << ";" << "coc.position_BRF[0]" << ";" << "coc.position_BRF[1]" << ";";
    file << "phase[0].get_state()" << ";" << "phase[1].get_state()" << ";" << "phase[2].get_state()" << ";" << "phase[3].get_state()" << ";";
    file << "phase[0].is_centered()" << ";" << "phase[1].is_centered()" << ";" << "phase[2].is_centered()" << ";" << "phase[3].is_centered()" << endl;

    for(float time_s=0.0f; time_s<6.0f; time_s+=cfg.dt)
    {

        // speed profil for simulation
        float vx {0.0f};
        float vy {0.0f};
        float wz {0.0f};

        if(time_s > 0.1f)
            vx = 0.05f;
        if(time_s > 2.5f)
            vx = 0.2f;
        if(time_s > 4.5f)
            vx = 0.0f;

        if(time_s > 1.0f)
            vy = 0.10f;
        if(time_s > 3.5f)
            vy = 0.0f;

        if(time_s > 3.0f)
            wz = 2.00f;
        if(time_s > 4.0f)
            wz = 0.0f;

        // gait computation

        vel_smo.update(vx,vy,wz);

        coc.update(vel_smo.get_vx(),vel_smo.get_vy(),vel_smo.get_wz());

        for(auto & p : phase)
            p.update(time_s,vel_smo.is_moving());

        file << time_s << ";" ;
        file << vx << ";" << vel_smo.get_vx() << ";";
        file << vy << ";" << vel_smo.get_vy() << ";";
        file << wz << ";" << vel_smo.get_wz() << ";";
        file << coc.distance << ";" << coc.angle << ";" << coc.position_BRF[0] << ";" << coc.position_BRF[1] << ";";
        file << phase[0].get_state() << ";" << phase[1].get_state() << ";" << phase[2].get_state() << ";" << phase[3].get_state() << ";";
        file << phase[0].is_centered() << ";" << phase[1].is_centered() << ";" << phase[2].is_centered() << ";" << phase[3].is_centered() << endl;
    }
    return 0;
}
