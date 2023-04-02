#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

#include <chrono>
using namespace std::chrono;

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
    joints j;

    /*
    mini_pupper::esp32_api servo;
    mini_pupper::parameters_control_instruction_format control;
    mini_pupper::parameters_control_acknowledge_format feedback;
    */

    /*
    // free joints
    for(size_t index=0; index<12; ++index)
    {
        control.goal_position[index]= 512;
        control.torque_enable[index] = 0;
    }
    int result = servo.update(control,feedback);
    std::cout << "result:" << result << std::endl;
    */

    // simulate timer-task
    ofstream file("gai.csv");
    file << "time_s" << ";" ;
    file << "vx" << ";" << "vel_smo.get_vx()" << ";";
    file << "vy" << ";" << "vel_smo.get_vy()" << ";";
    file << "wz" << ";" << "vel_smo.get_wz()" << ";";
    //file << "coc.distance" << ";" << "coc.angle" << ";" << "coc.position_BRF[0]" << ";" << "coc.position_BRF[1]" << ";";
    file << "phase[0].get_state()" << ";" << "phase[1].get_state()" << ";" << "phase[2].get_state()" << ";" << "phase[3].get_state()" << ";";
    file << "phase[0].is_centered()" << ";" << "phase[1].is_centered()" << ";" << "phase[2].is_centered()" << ";" << "phase[3].is_centered()" << endl;

    milliseconds t0_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    size_t counter {0};
    int errors {0};

    for(float time_s=0.0f; time_s<6.0f; time_s+=cfg.dt)
    {
        ++counter;

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
            l.update(vel_smo.get_vx(),vel_smo.get_vy(),vel_smo.get_wz());

        // compute in-place transformation
        Eigen::Matrix<float,3,4> feet_BRF;
        Eigen::Vector3f const translation { dx, dy, dz };
        Eigen::Matrix3f const rotation { rotation_from_euler(roll,pitch,yaw) };
        for(size_t leg_id=0; leg_id<4; ++leg_id)
            feet_BRF.col(leg_id) = rotation * ( leg[leg_id].get_foot_BRF() + translation );

        // compute IK, joint and servo position
        Eigen::Matrix<float,3,4> joint_position { kin.four_leg_inverse_kinematics_BRF(feet_BRF) };
        int servo_position[12] {0};
        j.position_setpoint(joint_position,servo_position);

        // ESP32
        /*
        for(size_t index=0; index<12; ++index)
        {
            control.goal_position[index]= position_setpoint[index];
            control.torque_enable[index] = 1;
        }
        int result = servo.update(control,feedback);
        if(result!=mini_pupper::API_OK) ++errors;
        */

        file << time_s << ";" ;
        file << vx << ";" << vel_smo.get_vx() << ";";
        file << vy << ";" << vel_smo.get_vy() << ";";
        file << wz << ";" << vel_smo.get_wz() << ";";
        //file << coc.distance << ";" << coc.angle << ";" << coc.position_BRF[0] << ";" << coc.position_BRF[1] << ";";
        file << phase[0].get_state() << ";" << phase[1].get_state() << ";" << phase[2].get_state() << ";" << phase[3].get_state() << ";";
        file << phase[0].is_centered() << ";" << phase[1].is_centered() << ";" << phase[2].is_centered() << ";" << phase[3].is_centered() << ";";

        file << leg[0].get_foot_BRF()[0] << ";" << leg[0].get_foot_BRF()[1] << ";" << leg[0].get_foot_BRF()[2] << endl;
    }

    milliseconds t1_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
    duration<float> difference = t1_ms - t0_ms;
    int const milliseconds = difference.count() * 1000;
    std::cout << "delay:" << milliseconds << "ms (erros count:" << errors << ")" << std::endl;
    std::cout << "frequency:" << (counter*1000.0/milliseconds) << " Hz" << std::endl;

    /*
    // free joints
    for(size_t index=0; index<12; ++index)
    {
        control.goal_position[index]= 512;
        control.torque_enable[index] = 0;
    }
    int result = servo.update(control,feedback);
    std::cout << "result:" << result << std::endl;
    */

    return 0;
}
