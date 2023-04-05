/* Authors :
 * - Hdumcke
 * - Pat92fr
 */
#ifndef _esp32_api_H
#define _esp32_api_H

#include <cstdint>

namespace mini_pupper
{
    enum
    {
        API_LL_ERROR        = -1,
        API_OK              =  0,
        API_TIME_OUT        =  1,
        API_INVALID_HEADER  =  2,
        API_INVALID_CHECKSUM=  3,

    };

    // control structure
    struct parameters_control_instruction_format
    {
        uint8_t torque_enable[12];
        uint16_t goal_position[12];
    };

    // feedbakc structure
    struct parameters_control_acknowledge_format
    {
        // SERVO feedback
        uint16_t present_position[12];
        int16_t present_load[12];
        // IMU data
        float ax;
        float ay;
        float az;
        float gx;
        float gy;
        float gz;
        // POWER SUPPLY data
        float voltage_V;
        float current_A;
    };

    struct esp32_api
    {
        esp32_api(char const * device = "/dev/ttyAMA1");
        ~esp32_api();

        int update(parameters_control_instruction_format & control, parameters_control_acknowledge_format & feedback) const;

    private:
        uint8_t _compute_checksum(uint8_t const buffer[]) const;
        bool _checksum(uint8_t const buffer[], uint8_t & expected_checksum) const;

        // device
        int _fd {-1};
    };

}

#endif //_esp32_api_H

/*
 * Get access to serial port
 *
 * sudo apt-get install raspi-config
 * sudo raspi-config
 * sudo cp -pr /boot/firmware/config.txt /boot/firmware/config.txt-orig
 * sudo cp -pr /boot/firmware/cmdline.txt /boot/firmware/cmdline.txt-orig
 * insert line ''dtoverlay=uart3'' in config.txt
 * dev/ttyAMA1 
 * https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts
 * sudo usermod -a -G dialout $USER
 * 
The easy way:

sudoedit /etc/udev/rules.d/50-myusb.rules
Save this text:

KERNEL=="ttyUSB[0-9]*",MODE="0666"
KERNEL=="ttyAMA[0-9]*",MODE="0666"
Unplug the device and replug it, and it should be read/write from any user!
*/