#include <iostream>

#include <chrono>
using namespace std::chrono;

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

    milliseconds t0_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    int result {0};
    int errors {0};
    for(int i=0;i<1000;++i)
    {
        result = proxy.update(control,feedback);
        if(result!=API_OK)
        {
            ++errors;
            std::cout << "update:" << result << std::endl;
        }
    }

    milliseconds t1_ms = duration_cast< milliseconds >(system_clock::now().time_since_epoch());

    duration<float> difference = t1_ms - t0_ms;
    int const milliseconds = difference.count() * 1000;

    std::cout << "1000x control-feedback delay:" << milliseconds << "ms (erros count:" << errors << ")" << std::endl;
    std::cout << "control-feedback frequency:" << (1000000.0/milliseconds) << " Hz" << std::endl;

    return 0;
}