// sudo pigpiod
// compile pigpiod C Interface: g++ -Wall -pthread -o calibrate calibrate.cpp -lpigpiod_if2 -lrt -lncurses
// http://abyz.me.uk/rpi/pigpio/pdif2.html#pigpio_start

#include "servo_imu_hat.h"

#include <ncurses.h>
#include <iostream>
#include <fstream>
using namespace std;

int position[12] {
    1500,   1500,   1500,
    1500,   1500,   1500,
    1500,   1500,   1500,
    1500,   1500,   1500
};

size_t current_index {0};

void show()
{
    move(0,0);
    for(size_t index=0;index<12;++index)
    {
        if(index==current_index)
            printw(">");
        else
            printw(" ");
        printw("%d:%d",(int)(index+1),position[index]);
        if(index==current_index)
            printw("<");
        else
            printw(" ");
        printw(" ");
    }
    printw("         ");
    refresh();

}

int main(int argc, char *argv[])
{
    mini_pupper::servo_imu_hat servo;
    mini_pupper::parameters_control_instruction_format control;

    // standby servo
    for(size_t channel = 0; channel<12; ++channel)
    {
        control.torque_enable[channel]=1;
        control.goal_position[channel]=position[channel];
    }
    servo.update(control);

    cout << "Manual calibration:" << endl;
    cout << " - left/right to change channel." << endl;
    cout << " - up/down to adjust position [1000,2000]." << endl;
    cout << " - R to reset position (1500)." << endl;
    cout << " - Q to exit." << endl;
    cout << endl;

    initscr();
    keypad(stdscr, TRUE); /* pour récupérer les événements du clavier */
    noecho(); /* pour masquer l'affichage du buffer stdin dans la console */
    cbreak();

    bool run {true};
    while(run)
    {
        show();
        int code = getch();
        //if(code==224)
        {
            //code = getch();
            switch(code)
            {
            case KEY_UP: // up
                //cout << "up ";
                position[current_index] = min(2350,position[current_index]+1);
                break;
            case KEY_DOWN: // down
                //cout << "down ";
                position[current_index] = max(650,position[current_index]-1);
                break;
            case KEY_RIGHT: // right
                //cout << "> ";
                if(current_index<11)
                    ++current_index;
                break;
            case KEY_LEFT: // left
                //cout << "< ";
                if(current_index>0)
                    --current_index;
                break;
            case 'r':
                position[current_index] = 1500;
                break;
            case 'q':
                run = false;
                break;
            default:
                break;
            }
        }

        // update position
        for(size_t channel = 0; channel<12; ++channel)
        {
            control.torque_enable[channel]=1;
            control.goal_position[channel]=position[channel];
        }    
        servo.update(control);

    }

    // free servo
    for(size_t channel = 0; channel<12; ++channel)
    {
        control.torque_enable[channel]=0;
        control.goal_position[channel]=1500;
    }    
    servo.update(control);

    // write to file
    {
        ofstream f {"calibration.txt"};
        for(size_t channel = 0; channel<12; ++channel)
        {
            f << position[channel] << endl;
        }
    }

    echo();
    endwin();
    return 0;
}
