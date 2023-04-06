// sudo pigpiod
// compile:  g++ -Wall -pthread -o test test.cpp -lpigpio -lrt -lpigpiod_if
// compile pigpiod C Interface: g++ -Wall -pthread -o test test.cpp -lpigpiod_if2 -lrt
// http://abyz.me.uk/rpi/pigpio/pdif2.html#pigpio_start

#include <iostream>
using namespace std;

#include <stdint.h>
#include <unistd.h>

#include <pigpio.h>
#include <pigpiod_if2.h>


namespace mini_pupper
{

    // control structure
    struct parameters_control_instruction_format
    {
        uint8_t torque_enable[12];
        uint16_t goal_position[12];
    };

    struct servo_hat_api
    {
        servo_hat_api():
        _pi(pigpio_start(nullptr,nullptr))
        {
            if(_pi>=0)
            {
                cout << "pigpio_start OK (" << _pi << ")" << endl;

                for(int const & gpio : pwm_gpio)
                {
                    int frequency { set_PWM_frequency(_pi,gpio,320) };
                    cout << "#" << gpio << " set_PWM_frequency (" << frequency << ")" << endl;

                    int range { set_PWM_range(_pi,gpio, 3125) }; 
                    cout << "#" << gpio << " set_PWM_range (" << range << ")" << endl;
                }
            }
            else
            {
                cout << "pigpio_start error (" << _pi << ")" << endl;
            }

        }

        ~servo_hat_api()
        {
            if(_pi>=0)
            {
                pigpio_stop(_pi);
                cout << "pigpio_stop (" << _pi << ")" << endl;
            }
        }

        void update(parameters_control_instruction_format & control) const
        {
            if(_pi>=0)
            {
                for(size_t channel = 0; channel<12; ++channel)
                {
                    int result {0};
                    if(control.torque_enable[channel]==1)
                        result = set_PWM_dutycycle(_pi,pwm_gpio[channel], control.goal_position[channel]);
                    else
                        result = set_PWM_dutycycle(_pi,pwm_gpio[channel], 0);
                    if(result)
                        cout << "#" << pwm_gpio[channel] << " set_PWM_dutycycle error (" << result << ")" << endl;
                }
            }
        }

    private:

        int const pwm_gpio[12] {5,6,12,13,16,19,20,21,26,23,24,25};

        // device
        int _pi {-1};
    };

}


int main(int argc, char *argv[])
{
    mini_pupper::servo_hat_api servo;

    mini_pupper::parameters_control_instruction_format control;

    // free servo
    for(size_t channel = 0; channel<12; ++channel)
    {
        control.torque_enable[channel]=0;
        control.goal_position[channel]=1500;
    }

    for(size_t index=0; index<2; ++index)
    {
        size_t channel {7};
        control.torque_enable[channel]=1;
        control.goal_position[channel]=1500;
        servo.update(control);
        sleep(1);
        control.goal_position[channel]=1000;
        servo.update(control);
        sleep(1);        
        control.goal_position[channel]=1500;
        servo.update(control);
        sleep(1);
        control.goal_position[channel]=2000;
        servo.update(control);
        sleep(1);                
    }

    // free servo
    for(size_t channel = 0; channel<12; ++channel)
    {
        control.torque_enable[channel]=0;
        control.goal_position[channel]=1500;
    }    
}
