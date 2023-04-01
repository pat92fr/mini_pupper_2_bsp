#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
using namespace std;

#include "mini_pupper.h"
using namespace mini_pupper;

// TODO : velocity smoother : config (acc x/y, acc wz, min/max vx,vy,wz) + velocities (vx,vy,wz) ==> (walk, vx, vy, wz filtered/capped)

// TODO : config(height) + (state, alpha, vx, vy, wz filtered) ==> dX,dY,dZ around STANCE POSE

// CoM dynamic control

int main()
{
    cout << "Hello world!" << endl;

    config cfg;
    vector<gait_phase> phase {
        { cfg, LEG_FR },
        { cfg, LEG_FL },
        { cfg, LEG_RR },
        { cfg, LEG_RL }
    };


    ofstream file("gai.csv");
    for(float time_s=0.0f; time_s<3.0f; time_s+=0.005)
    {
        bool walk { time_s > 0.5f && time_s < 2.5f };
        for(auto & p : phase)
            p.update(time_s,walk);

        //file << time_s << ";" << phase[0]._state << ";" << phase[0]._alpha << ";" << phase[1]._state << ";" << phase[1]._alpha << endl;
        file << time_s << ";" << phase[0].get_state() << ";" << phase[1].get_state() << ";" << phase[2].get_state() << ";" << phase[3].get_state() << endl;
    }
    return 0;
}
