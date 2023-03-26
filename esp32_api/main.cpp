#include <iostream>

#include "esp32-api.h"
using namespace mini_pupper;

esp32_api proxy;

int main(int argc, char *argv[])
{
    parameters_control_instruction_format control;
    parameters_control_acknowledge_format feedback;

    // Initialize all control/goal_position to neutral
    uint16_t const neutral_pos {512};
    for(auto & goal_position : control.goal_position)
    {
        goal_position = neutral_pos;
    }

    // Initialize all control/torque_switch to enable
    for(auto & torque_enable : control.torque_enable)
    {
        torque_enable = 0;
    }

    for(int i=0;i<10;++i)
    {
        int result = proxy.update(control,feedback);
        std::cout << "update:" << result << std::endl;
    }

    return 0;
}