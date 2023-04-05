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

int main(int argc, char *argv[])
{
    int pi = pigpio_start(nullptr,nullptr);
    if(pi>=0)
        cout << "pigpio_start OK (" << pi << ")" << endl;
    else
    {
        cout << "pigpio_start error (" << pi << ")" << endl;
        return -1;
    }

    int frequency { set_PWM_frequency(pi,21,320) };
    cout << "set_PWM_frequency (" << frequency << ")" << endl;

    int range { set_PWM_range(pi,21, 3125) }; 
    cout << "set_PWM_range (" << range << ")" << endl;


    int result { 0 };
    for(size_t index=0; index<2; ++index)
    {
        result = set_PWM_dutycycle(pi,21, 1500);
        if(result)
            cout << "set_PWM_dutycycle error (" << result << ")" << endl;
        cout << "PWM: " << get_PWM_dutycycle(pi,21) << endl;
        sleep(1);
        result = set_PWM_dutycycle(pi,21, 1000);
        if(result)
            cout << "set_PWM_dutycycle error (" << result << ")" << endl;
        cout << "PWM: " << get_PWM_dutycycle(pi,21) << endl;
        sleep(1);
        result = set_PWM_dutycycle(pi,21, 1500);
        if(result)
            cout << "set_PWM_dutycycle error (" << result << ")" << endl;
        cout << "PWM: " << get_PWM_dutycycle(pi,21) << endl;
        sleep(1);
        result = set_PWM_dutycycle(pi,21, 2000);
        if(result)
            cout << "set_PWM_dutycycle error (" << result << ")" << endl;
        cout << "PWM: " << get_PWM_dutycycle(pi,21) << endl;
        sleep(1);
    }
    result = set_PWM_dutycycle(pi,21, 0);
    if(result)
        cout << "set_PWM_dutycycle error (" << result << ")" << endl;
    sleep(1);


    pigpio_stop(pi);

}


/*
    if(result)
        cout << "set_PWM_dutycycle (" << result << ")" << endl;

*/