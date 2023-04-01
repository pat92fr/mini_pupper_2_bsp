#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
using namespace std;

struct t_gait_config
{
    float period_s {0.500f};
    float stance_s {0.300f};
    float stance_offset[4] {
        0.0f,
        0.5f,
        0.0f,
        0.5f
    };
};

enum e_leg
{
    LEG_FR = 0,
    LEG_FL = 1,
    LEG_RR = 2,
    LEG_RL = 3
};

enum e_phase_state
{
    PHASE_STANCE = 0,
    PHASE_SWING
};

struct t_phase
{
    t_phase(t_gait_config & config, e_leg leg) :
        _config( config),
        _leg(leg)
    {};

    void update(
        float time_s,
        bool walk = false
    )
    {
        float time_mod_s { fmod(time_s-_config.stance_offset[_leg]*_config.period_s+0.5f*_config.stance_s,_config.period_s) };
        if(time_mod_s<_config.stance_s)
        {
            _state = PHASE_STANCE;
            _alpha = time_mod_s/_config.stance_s;
        }
        else
        {
            _state = PHASE_SWING;
            _alpha = (time_mod_s-_config.stance_s)/(_config.period_s-_config.stance_s);
        }
    }

    t_gait_config & _config;
    e_leg _leg;
    e_phase_state _state {PHASE_STANCE};    // STANCE or SWING
    float _alpha {0};        // [0,1[
};




int main()
{
    cout << "Hello world!" << endl;

    ofstream file("gai.csv");

    t_gait_config config;
    vector<t_phase> phase {
        { config, LEG_FR },
        { config, LEG_FL },
        { config, LEG_RR },
        { config, LEG_RL }
    };


    for(float time_s=0.0f; time_s<3.0f; time_s+=0.005)
    {
        for(auto & p : phase)
            p.update(time_s,true);
        //file << time_s << ";" << phase[0]._state << ";" << phase[0]._alpha << ";" << phase[1]._state << ";" << phase[1]._alpha << endl;
        file << time_s << ";" << phase[0]._state << ";" << phase[1]._state << ";" << phase[2]._state << ";" << phase[3]._state << endl;
    }
    return 0;
}
