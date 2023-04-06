#include <iostream>
#include <fstream>
#include <iostream>
#include <conio.h>
#include <cstdlib>
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
    for(size_t index=0;index<12;++index)
    {
        if(index==current_index)
            cout << ">";
        else
            cout << " ";
        cout << (index+1) << ":" << position[index];
        if(index==current_index)
            cout << "<";
        else
            cout << " ";
        cout << " ";
    }
    cout << "              \r";

}
int main(int argc, char **argv)
{
    cout << "Manual calibration:" << endl;
    cout << " - left/right to change channel." << endl;
    cout << " - up/down to adjust position [1000,2000]." << endl;
    cout << " - R to reset position (1500)." << endl;
    cout << " - Q to exit." << endl;
    cout << endl;

    while(true)
    {
        show();
        int code = getch();
        if(code==224)
        {
            code = getch();
            switch(code)
            {
            case 72: // up
                //cout << "up ";
                position[current_index] = min(2000,position[current_index]+1);
                break;
            case 80: // down
                //cout << "down ";
                position[current_index] = max(1000,position[current_index]-1);
                break;
            case 77: // right
                //cout << "> ";
                if(current_index<11)
                    ++current_index;
                break;
            case 75: // left
                //cout << "< ";
                if(current_index>0)
                    --current_index;
                break;
            default:
                break;
            }
        }
        else if(code=='r')
        {
            position[current_index] = 1500;
        }
        else if(code=='q')
        {
            return 0;
        }
    }
    return 0;
}
