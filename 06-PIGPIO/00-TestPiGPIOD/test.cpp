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

    enum mpu9250_register
    {
        SAMPLE_RATE_DIVIDER = 0x19,
        CONFIG = 0x1A,
        GYRO_CONFIG = 0x1B,
        ACCEL_CONFIG = 0x1C,
        ACCEL_CONFIG_2 = 0x1D,
        INT_BYP_CFG = 0x37,
        INT_ENABLE = 0x38,
        INT_STATUS = 0x3A,
        ACCEL_X_HIGH = 0x3B,
        ACCEL_X_LOW = 0x3C,
        ACCEL_Y_HIGH = 0x3D,
        ACCEL_Y_LOW = 0x3E,
        ACCEL_Z_HIGH = 0x3F,
        ACCEL_Z_LOW = 0x40,
        TEMP_HIGH = 0x41,
        TEMP_LOW = 0x42,
        GYRO_X_HIGH = 0x43,
        GYRO_X_LOW = 0x44,
        GYRO_Y_HIGH = 0x45,
        GYRO_Y_LOW = 0x46,
        GYRO_Z_HIGH = 0x47,
        GYRO_Z_LOW = 0x48,
        PWR_MGMT_1 = 0x6B,
        PWR_MGMT_2 = 0x6C,
        WHO_AM_I = 0x75
    };

    struct servo_hat_api
    {
        servo_hat_api():
        _pi(pigpio_start(nullptr,nullptr))
        {
            if(_pi>=0)
            {
                cout << "pigpio_start OK (" << _pi << ")" << endl;

                // initialize 12 PWM outputs and set to idle
                for(int const & gpio : pwm_gpio)
                {
                    int const frequency { set_PWM_frequency(_pi,gpio,320) };
                    cout << "#" << gpio << " set_PWM_frequency (" << frequency << ")" << endl;

                    int const range { set_PWM_range(_pi,gpio, 3125) }; 
                    cout << "#" << gpio << " set_PWM_range (" << range << ")" << endl;

                    int const result { set_PWM_dutycycle(_pi,gpio, 0) };
                    cout << "#" << gpio << " set_PWM_dutycycle (" << result << ")" << endl;
                }

                // initialize I2C (bus #1) MPU9250 @68h
                _i2c_handle = i2c_open(_pi,1,0x68,0);
                cout << "i2c_open (" << _i2c_handle << ")" << endl;
                int result {0};
                if(_i2c_handle>=0)
                {
                    // check ID
                    int const byte { i2c_read_byte_data(_pi,_i2c_handle,WHO_AM_I) };
                    cout << "@WHO_AM_I i2c_read_byte_data (" << byte << ")" << endl;
                    
                    // Reset and activate XYZ Acc and Gyro
                    result = i2c_write_byte_data(_pi,_i2c_handle,PWR_MGMT_1,0x80);
                    cout << "i2c_write_byte_data PWR_MGMT_1=10000000b (" << result << ")" << endl;
                    usleep(50000);

                    // Auto selects the best available clock source
                    result = i2c_write_byte_data(_pi,_i2c_handle,PWR_MGMT_1,0x01);
                    cout << "i2c_write_byte_data PWR_MGMT_1=00000001b (" << result << ")" << endl;

                    // Sampling rate : 333z
                    result = i2c_write_byte_data(_pi,_i2c_handle,SAMPLE_RATE_DIVIDER,0x02);
                    cout << "i2c_write_byte_data SAMPLE_RATE_DIVIDER=00000011b (" << result << ")" << endl;

                    // Gyro Configuration : BW=184Hz Fs=1KHz
                    result = i2c_write_byte_data(_pi,_i2c_handle,CONFIG,0x01);
                    cout << "i2c_write_byte_data CONFIG=00000001b (" << result << ")" << endl;


                    // Gyro Configuration : 2000dps 250Hz 
                    result = i2c_write_byte_data(_pi,_i2c_handle,GYRO_CONFIG,0x18);
                    cout << "i2c_write_byte_data GYRO_CONFIG=00011000b (" << result << ")" << endl;

                    // Accelerometer Configuration : 16g
                    result = i2c_write_byte_data(_pi,_i2c_handle,ACCEL_CONFIG,0x18);
                    cout << "i2c_write_byte_data ACCEL_CONFIG=0001100b (" << result << ")" << endl;

                    // Accelerometer Configuration 2 : rate to 1 kHz and bandwidth to 41 Hz
                    result = i2c_write_byte_data(_pi,_i2c_handle,ACCEL_CONFIG_2,0x03);
                    cout << "i2c_write_byte_data ACCEL_CONFIG_2=00000011b (" << result << ")" << endl;

                    // Configure INT GPIO pin
                    result = set_mode(_pi, 4, PI_INPUT);
                    cout << "set_mode pin 4=INPUT (" << result << ")" << endl;

                    // Config INT is default (active high, push-pull, 50us pulse)

                    // Enable INT
                    result = i2c_write_byte_data(_pi,_i2c_handle,INT_ENABLE,0x01);
                    cout << "i2c_write_byte_data INT_ENABLE=00000001b (" << result << ")" << endl;

                    // Wake-up
                    result = i2c_write_byte_data(_pi,_i2c_handle,PWR_MGMT_1,0x00);
                    cout << "i2c_write_byte_data PWR_MGMT_1=00000000b (" << result << ")" << endl;

                    result = callback_ex(_pi, 4, RISING_EDGE, servo_hat_callback_i2c, this);
                    cout << "i2c_write_byte_data PWR_MGMT_1=00000000b (" << result << ")" << endl;
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
                i2c_close(_pi,_i2c_handle);
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

        static void servo_hat_callback_i2c(int pi, unsigned user_gpio, unsigned level, uint32_t tick, void * userdata)
        {
            servo_hat_api * hat = (servo_hat_api*)userdata;
            //cout << ".";

            char data[6+2+6];
            int const result { i2c_read_i2c_block_data(hat->_pi,hat->_i2c_handle,ACCEL_X_HIGH, data, 14) };
            if(result!=14)
                cout << 'x';

            int16_t const accel_temp[3] = 
            {
                (int16_t) (((int16_t)data[0] << 8) | data[1]  ), 
                (int16_t) (((int16_t)data[2] << 8) | data[3]  ), 
                (int16_t) (((int16_t)data[4] << 8) | data[5]  )
            };
            int16_t const gyro_temp[3] = 
            {
                (int16_t) (((int16_t)data[8] << 8) | data[9]  ), 
                (int16_t) (((int16_t)data[10] << 8) | data[11]  ), 
                (int16_t) (((int16_t)data[12] << 8) | data[13]  )
            };            
            
            hat->_ax = 1.0f/2048.0f * (float)accel_temp[0];
            hat->_ay = 1.0f/2048.0f * (float)accel_temp[1];
            hat->_az = 1.0f/2048.0f * (float)accel_temp[2];
            hat->_gx = 1.0f/16.4f * (float)gyro_temp[0];
            hat->_gy = 1.0f/16.4f * (float)gyro_temp[1];
            hat->_gz = 1.0f/16.4f * (float)gyro_temp[2];

            static size_t counter {0};
            if(counter++%333==0)
                cout << hat->_ax << " " << hat->_ay << " " << hat->_az << " " << hat->_gx << " " << hat->_gy << " " << hat->_gz << " " << endl;
        }

    private:

        int const pwm_gpio[12] {21,20,26,16,19,13,12,6,5,25,24,23}; // PCB order from ETHERNET to USB-C

        // I2C BUS #1 : SCL (3) SDA (2) INT (4)

        // device
        int _pi {-1};
        int _i2c_handle {-1};

        // IMU
        float _ax {0.0f};
        float _ay {0.0f};
        float _az {0.0f};
        float _gx {0.0f};
        float _gy {0.0f};
        float _gz {0.0f};
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
    servo.update(control);

    for(size_t index=0; index<2; ++index)
    {
        size_t channel {11};
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
    servo.update(control);
}
