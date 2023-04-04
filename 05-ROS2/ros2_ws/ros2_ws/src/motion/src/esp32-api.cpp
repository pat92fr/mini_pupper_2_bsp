//static char const * version = PROJECT_VER;

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <termios.h>

static bool const print_debug     {false};
static bool const print_debug_max {false};

// host instruction code
#define INST_CONTROL 0x01   // Host sends servo position setpoints, ESP32 replies with servo feedback, attitude, ....

#include "esp32-api.h"
using namespace mini_pupper;

esp32_api::esp32_api(char const * device) :
_fd(-1)
{
    // reference : https://www.pololu.com/docs/0J73/15.5

    // Open serial device
    _fd = open(
        device,           // UART device
        O_RDWR | O_NOCTTY   // Read-Write access + Not the terminal of this process
    );
    if (_fd < 0)
    {
        printf("%s: failed to open UART device\n", __func__);
        return;
    }

    // Flush away any bytes previously read or written.
    int result = tcflush(_fd, TCIOFLUSH);
    if (result)
    {
        printf("%s: failed to flush\r\n", __func__);
        close(_fd);
        _fd = -1;
        return;
    }

    // Get the current configuration of the serial port.
    struct termios options;
    result = tcgetattr(_fd, &options);
    if (result)
    {
        printf("%s: failed to allocate UART TTY instance\n", __func__);
        close(_fd);
        _fd = -1;
        return;
    }

    // note : Les fonctions termios établissent une interface générale 
    //        sous forme de terminal, permettant de contrôler les ports 
    //        de communication asynchrone.  

    // Turn off any options that might interfere with our ability to send and
    // receive raw binary bytes.
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF); // IGNPAR ?
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Set up timeouts: Calls to read() will return as soon as there is
    // at least one byte available or when 100 ms has passed.
    // http://unixwiz.net/techtips/termios-vmin-vtime.html
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;

    // Set baud rate
    cfsetospeed(&options, B3000000);

    // Apply options
    result = tcsetattr(_fd, TCSANOW, &options);
    if (result)
    {
        printf("%s: failed to set attributes\r\n", __func__);
        close(_fd);
        _fd = -1;
        return;
    }
}

esp32_api::~esp32_api()
{
    close(_fd);
}

uint8_t esp32_api::_compute_checksum(uint8_t const buffer[]) const
{
    size_t const frame_size { (size_t)(buffer[3]+4) };
    uint8_t chk_sum = 0;
    for(size_t index=2; index<(frame_size-1); ++index)
        chk_sum += buffer[index];
    return ~chk_sum;
}

bool esp32_api::_checksum(uint8_t const buffer[], uint8_t & expected_checksum) const
{
    size_t const frame_size { (size_t)(buffer[3]+4) };
    uint8_t const received_checksum {buffer[frame_size-1]};
    expected_checksum = _compute_checksum(buffer);
    return received_checksum==expected_checksum;
}

int esp32_api::update(parameters_control_instruction_format & control, parameters_control_acknowledge_format & feedback) const
{
    if(_fd<0) return API_LL_ERROR;

    // heart-beat
    if (print_debug_max)
    {
        printf(".");
    }

    // Flush away any bytes previously read or written.
    int result = tcflush(_fd, TCIOFLUSH);
    if (result)
    {
        printf("%s: failed to flush\r\n", __func__);
        return API_LL_ERROR;
    }

    // Compute the size of the payload (parameters length + 2)
    size_t const tx_payload_length { sizeof(parameters_control_instruction_format) + 2 };

    // Compute the size of the frame
    size_t const tx_buffer_size { tx_payload_length + 4 };

    // Build the frame
    uint8_t tx_buffer[tx_buffer_size]
    {
        0xFF,               // header
        0xFF,               // header
        0x01,               // default ID
        tx_payload_length,  // length
        INST_CONTROL        // instruction
    };
    memcpy(tx_buffer+5,&control,sizeof(parameters_control_instruction_format));

    // Checksum
    tx_buffer[tx_buffer_size-1] = _compute_checksum(tx_buffer);

    // Send
    result = write(_fd, (char *)tx_buffer, tx_buffer_size);
    if(result != (ssize_t)tx_buffer_size)
    {
        printf("failed to write to port (%d,%ld,fd=%d)\r\n",result,tx_buffer_size,_fd);
        return API_LL_ERROR;
    }
    if (print_debug_max)
    {
        printf("uart writen:%d\r\n",result);
    }

    /*
    result = tcdrain(fd);
    if(result)
    {
        printf("failed to drain port");
        close(fd);
        exit(EXIT_FAILURE);
    }
    */

    // Read buffer
    // If we do not get data within 100ms we assume ESP32 stopped sending and we send again
    size_t const rx_buffer_size { 128 };
    uint8_t rx_buffer[rx_buffer_size] {0};
    size_t const expected_lenght {4 + 1 + sizeof(parameters_control_acknowledge_format) + 1}; // header + status + pos + load + chksum
    size_t received_length {0};
    while(received_length<expected_lenght)
    {
        ssize_t read_length = read(
            _fd,
            (char*)(rx_buffer+received_length),
            expected_lenght-received_length
        );
        if(read_length<0)
        {
            printf("failed to read from port\r\n");
            return API_LL_ERROR;
        }
        if (print_debug_max)
        {
            printf("uart read:%lu, waiting for %lu/%lu\r\n",read_length, received_length,expected_lenght);
        }
        if(read_length==0)
        {
            // time out
            break;
        }
        received_length += read_length;
    }

    // not enough data received ?
    if(received_length<expected_lenght) return API_TIME_OUT;

    /*
    * Decode a CONTROL ACK frame
    */

    bool const rx_header_check {
                (rx_buffer[0]==0xFF)
            &&  (rx_buffer[1]==0xFF)
            &&  (rx_buffer[2]==0x01) // my ID
            &&  (rx_buffer[3]<=(rx_buffer_size-4)) // keep the header in the rx buffer
    };
    if(!rx_header_check)
    {
        // log
        if (print_debug)
        {
            printf("RX frame error : header invalid!\r\n");
        }
        // next
        return API_INVALID_HEADER;
    }

    // checksum
    uint8_t expected_checksum {0};
    bool const rx_checksum {_checksum(rx_buffer,expected_checksum)};

    // waiting for a valid instruction and checksum...
    bool const rx_payload_checksum_check {
                (rx_buffer[4]==0x00)
            &&  rx_checksum
    };
    if(!rx_payload_checksum_check)
    {
        // log
        if (print_debug)
        {
                printf("RX frame error : bad instruction [%d] or checksum [received:%d,expected:%d]!\r\n",rx_buffer[4],rx_buffer[rx_buffer[3]+3],expected_checksum);
        }
        // next
        return API_INVALID_CHECKSUM;
    }

    // decode parameters
    memcpy(&feedback,rx_buffer+5,sizeof(parameters_control_acknowledge_format));

    // log
    if (print_debug)
    {
        printf("Present Position: %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
        feedback.present_position[0],feedback.present_position[1],feedback.present_position[2],
        feedback.present_position[3],feedback.present_position[4],feedback.present_position[5],
        feedback.present_position[6],feedback.present_position[7],feedback.present_position[8],
        feedback.present_position[9],feedback.present_position[10],feedback.present_position[11]
        );
        printf("            Load: %d %d %d %d %d %d %d %d %d %d %d %d\r\n",
        feedback.present_load[0],feedback.present_load[1],feedback.present_load[2],
        feedback.present_load[3],feedback.present_load[4],feedback.present_load[5],
        feedback.present_load[6],feedback.present_load[7],feedback.present_load[8],
        feedback.present_load[9],feedback.present_load[10],feedback.present_load[11]
        );

        printf("Attitude:  ax:%.3f  ay:%.3f  az:%.3f  gx:%.3f  gy:%.3f  gz:%.3f\r\n",
            feedback.ax, feedback.ay, feedback.az,
            feedback.gx, feedback.gy, feedback.gz
        );
        printf("Power:  %.3fV  %.3fA\r\n", feedback.voltage_V, feedback.current_A);
    }

    return API_OK; // ok
}

