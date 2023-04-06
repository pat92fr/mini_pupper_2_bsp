// sudo pigpiod
// compile pigpiod C Interface: g++ -Wall -pthread -o test test.cpp -lpigpiod_if2 -lrt
// compile pigpiod C Interface: g++ -Wall -pthread -o test  FusionAhrs.o FusionCompass.o FusionOffset.o test.cpp -lpigpiod_if2 -lrt -lncurses -std=c++20

// http://abyz.me.uk/rpi/pigpio/pdif2.html#pigpio_start

#include <iostream>
using namespace std;

#include "servo_imu_hat.h"

int main(int argc, char *argv[])
{
    mini_pupper::servo_imu_hat servo;

    mini_pupper::parameters_control_instruction_format control;

    // free servo
    for(size_t channel = 0; channel<12; ++channel)
    {
        control.torque_enable[channel]=0;
        control.goal_position[channel]=1500;
    }
    servo.update(control);
/*
    for(size_t index=0; index<2; ++index)
    {
        size_t channel {0};
        control.torque_enable[channel]=1;
        control.goal_position[channel]=1500;
        servo.update(control);
        sleep(1);
        control.goal_position[channel]=1400;
        servo.update(control);
        sleep(1);        
        control.goal_position[channel]=1500;
        servo.update(control);
        sleep(1);
        control.goal_position[channel]=1600;
        servo.update(control);
        sleep(1);                
    }
*/
    // free servo
    for(size_t channel = 0; channel<12; ++channel)
    {
        control.torque_enable[channel]=0;
        control.goal_position[channel]=1500;
    }    
    servo.update(control);
    sleep(60); 
}
