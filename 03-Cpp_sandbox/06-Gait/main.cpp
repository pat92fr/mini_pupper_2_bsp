#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

#include "mini_pupper.h"
using namespace mini_pupper;


// Height & CoM dynamic control

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
    kinematic kin;
    vector<leg_trajectory> leg {
        { cfg, LEG_FR, phase[LEG_FR], kin },
        { cfg, LEG_FL, phase[LEG_FL], kin },
        { cfg, LEG_RR, phase[LEG_RR], kin },
        { cfg, LEG_RL, phase[LEG_RL], kin }
    };



    // simulate timer-task
    ofstream file("gai.csv");
    file << "time_s" << ";" ;
    file << "vx" << ";" << "vel_smo.get_vx()" << ";";
    file << "vy" << ";" << "vel_smo.get_vy()" << ";";
    file << "wz" << ";" << "vel_smo.get_wz()" << ";";
    //file << "coc.distance" << ";" << "coc.angle" << ";" << "coc.position_BRF[0]" << ";" << "coc.position_BRF[1]" << ";";
    file << "phase[0].get_state()" << ";" << "phase[1].get_state()" << ";" << "phase[2].get_state()" << ";" << "phase[3].get_state()" << ";";
    file << "phase[0].is_centered()" << ";" << "phase[1].is_centered()" << ";" << "phase[2].is_centered()" << ";" << "phase[3].is_centered()" << endl;

    for(float time_s=0.0f; time_s<6.0f; time_s+=cfg.dt)
    {

        // speed profile for simulation
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
            vy = 0.0f;
        if(time_s > 3.5f)
            vy = 0.0f;

        if(time_s > 3.0f)
            wz = 1.0f;
        if(time_s > 4.0f)
            wz = 0.0f;

        float dx {0.0f};
        float dy {0.0f};
        float dz {0.0f};

        float pitch {0.0f};
        float roll {0.0f};
        float yaw {0.0f};

        // gait computation

        vel_smo.update(vx,vy,wz);


        for(auto & p : phase)
            p.update(time_s,vel_smo.is_moving());
        for(auto & l : leg)
            l.update(
                vel_smo.get_vx(),vel_smo.get_vy(),vel_smo.get_wz(),
                 dx,dy,dz,
                 pitch,roll,yaw
            );

        file << time_s << ";" ;
        file << vx << ";" << vel_smo.get_vx() << ";";
        file << vy << ";" << vel_smo.get_vy() << ";";
        file << wz << ";" << vel_smo.get_wz() << ";";
        //file << coc.distance << ";" << coc.angle << ";" << coc.position_BRF[0] << ";" << coc.position_BRF[1] << ";";
        file << phase[0].get_state() << ";" << phase[1].get_state() << ";" << phase[2].get_state() << ";" << phase[3].get_state() << ";";
        file << phase[0].is_centered() << ";" << phase[1].is_centered() << ";" << phase[2].is_centered() << ";" << phase[3].is_centered() << ";";

        file << leg[0].get_foot_BRF()[0] << ";" << leg[0].get_foot_BRF()[1] << ";" << leg[0].get_foot_BRF()[2] << endl;
    }
    return 0;
}
