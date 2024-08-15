/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file serial_port.cpp
 *
 * @brief Serial interface functions
 *
 * Functions for opening, closing, reading and writing via serial ports
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */


// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port_mav.h"

using namespace std;

#if 1
#define SERIAL_RDWR_SIZE MAVLINK_MAX_PACKET_LEN
#else
#define SERIAL_RDWR_SIZE 300
#endif

// ----------------------------------------------------------------------------------
//   Serial Port Manager Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Con/De structors
// ------------------------------------------------------------------------------
Serial_Port_Mav::
Serial_Port_Mav(const char *uart_name_ , int baudrate_)
{
    initialize_defaults();
    MAVLINK_SERIAL_LOG("uart name: %s", uart_name_);
    uart_name = uart_name_;
    baudrate  = baudrate_;
}

Serial_Port_Mav::
Serial_Port_Mav()
{
    initialize_defaults();
}

Serial_Port_Mav::
~Serial_Port_Mav()
{
    // destroy mutex
    pthread_mutex_destroy(&port_lock);
}

void
Serial_Port_Mav::
initialize_defaults()
{
    // Initialize attributes
    debug  = false;
    fd     = -1;
    is_open = false;

    uart_name = (char*)"/dev/ttyUSB0";
    baudrate  = 57600;

    // Start mutex
    int result = pthread_mutex_init(&port_lock, NULL);
    if ( result != 0 )
    {
        printf("\n mutex init failed\n");
        throw 1;
    }
}


// ------------------------------------------------------------------------------
//   Read from Serial
// ------------------------------------------------------------------------------
int
Serial_Port_Mav::
read_message(queue<mavlink_message_t> &read_msg_q)
{
    mavlink_status_t status;
    mavlink_message_t message;
    uint8_t          msgReceived = 0;
    char             buf[SERIAL_RDWR_SIZE] = {0};

    // --------------------------------------------------------------------------
    //   READ FROM PORT
    // --------------------------------------------------------------------------

    // this function locks the port during read
    int result = _read_port(buf, SERIAL_RDWR_SIZE);

    // --------------------------------------------------------------------------
    //   PARSE MESSAGE
    // --------------------------------------------------------------------------
    if (result > 0)
    {
        for(int i = 0; i < result; ++i)
        {
            // the parsing
            msgReceived = mavlink_parse_char(mavlink_channel, buf[i], &message, &status);

            if(msgReceived){
                read_msg_q.push(message);
                msgReceived = 0;
                // printf("%s got msg id: %d\n", __func__, message.msgid);
            }
        }
    }
    else // Couldn't read from port
    {
        fprintf(stderr, "ERROR: Could not read from port\n");
        return -1;
    }

    // --------------------------------------------------------------------------
    //   DEBUGGING REPORTS
    // --------------------------------------------------------------------------
    if(result > 0 && debug)
    {
        // Report info
        printf("Received %d mavlink message(s) from serial\n", result);
    }

    // Done!
    return result;
}

// ------------------------------------------------------------------------------
//   Write to Serial
// ------------------------------------------------------------------------------
int
Serial_Port_Mav::
write_message(const mavlink_message_t &message)
{
    char buf[SERIAL_RDWR_SIZE];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);

    // Write buffer to serial port, locks port while writing
    int bytesWritten = _write_port(buf, len);

    return bytesWritten;
}


// ------------------------------------------------------------------------------
//   Open Serial Port
// ------------------------------------------------------------------------------
/**
 * throws EXIT_FAILURE if could not open the port
 */
void
Serial_Port_Mav::
start()
{
    bool success = false;

    // --------------------------------------------------------------------------
    //   OPEN PORT
    // --------------------------------------------------------------------------
    MAVLINK_SERIAL_LOG("OPEN PORT");
    fd = _open_port(uart_name);
    // Check success
    if (fd == -1)
    {
        MAVLINK_SERIAL_LOG("failure, could not open port.");
        exit(-1);
    }

    // --------------------------------------------------------------------------
    //   SETUP PORT
    // --------------------------------------------------------------------------
    success = _setup_port(baudrate, 8, 1, false, false);

    // --------------------------------------------------------------------------
    //   CHECK STATUS
    // --------------------------------------------------------------------------
    if (!success)
    {
        MAVLINK_SERIAL_LOG("failure, could not configure port.");
        exit(-1);
    }
    if (fd <= 0)
    {
        MAVLINK_SERIAL_LOG("Connection attempt to port %s with %d baud, 8N1 failed, exiting.", uart_name, baudrate);
        exit(-1);
    }

    // --------------------------------------------------------------------------
    //   CONNECTED!
    // --------------------------------------------------------------------------
    MAVLINK_SERIAL_LOG("Connected to %s with %d baud, 8 data bits, no parity, 1 stop bit (8N1)", uart_name, baudrate);
    lastStatus.packet_rx_drop_count = 0;

    is_open = true;
    printf("\n");

    return;

}


// ------------------------------------------------------------------------------
//   Close Serial Port
// ------------------------------------------------------------------------------
void
Serial_Port_Mav::
stop()
{
    int result = -1;
    MAVLINK_SERIAL_LOG("CLOSE PORT");

    result = close(fd);
    if (result)
        fprintf(stderr,"WARNING: Error on port close (%i)\n", result );

    is_open = false;
    printf("\n");

}

/* Functions to set/get mavlink channel and version */

void Serial_Port_Mav::set_mavlink_channel(int _ch) {
    mavlink_channel = _ch;
}

int Serial_Port_Mav::get_mavlink_channel() {
    return mavlink_channel;
}

void Serial_Port_Mav::set_mavlink_version(int _ver) {
    mavlink_version = _ver;
    mavlink_set_proto_version(mavlink_channel, mavlink_version);
}

int Serial_Port_Mav::get_mavlink_version() {
    return mavlink_version;
}

// ------------------------------------------------------------------------------
//   Helper Function - Open Serial Port File Descriptor
// ------------------------------------------------------------------------------
// Where the actual port opening happens, returns file descriptor 'fd'
int
Serial_Port_Mav::
_open_port(const char* port)
{
    // Open serial port
    // O_RDWR - Read and write
    // O_NOCTTY - Ignore special chars like CTRL-C
    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    // Check for Errors
    if (fd == -1)
    {
        /* Could not open the port. */
        return(-1);
    }
    else // Finalize
    {
        fcntl(fd, F_SETFL, 0);
    }

    // Done!
    return fd;
}

// ------------------------------------------------------------------------------
//   Helper Function - Setup Serial Port
// ------------------------------------------------------------------------------
// Sets configuration, flags, and baud rate
bool
Serial_Port_Mav::
_setup_port(int baud, int data_bits, int stop_bits, bool parity, bool hardware_control)
{
    // Check file descriptor
    if(!isatty(fd))
    {
        fprintf(stderr, "\nERROR: file descriptor %d is NOT a serial port\n", fd);
        return false;
    }

    // Read file descritor configuration
    struct termios config;
    if(tcgetattr(fd, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not read configuration of fd %d\n", fd);
        return false;
    }

    // Input flags - Turn off input processing
    // convert break to null byte, no CR to NL translation,
    // no NL to CR translation, don't mark parity errors or breaks
    // no input parity check, don't strip high bit off,
    // no XON/XOFF software flow control
    config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
                        INLCR | PARMRK | INPCK | ISTRIP | IXON);

    // Output flags - Turn off output processing
    // no CR to NL translation, no NL to CR-NL translation,
    // no NL to CR translation, no column 0 CR suppression,
    // no Ctrl-D suppression, no fill characters, no case mapping,
    // no local output processing
    config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
                         ONOCR | OFILL | OPOST);

    #ifdef OLCUC
        config.c_oflag &= ~OLCUC;
    #endif

    #ifdef ONOEOT
        config.c_oflag &= ~ONOEOT;
    #endif

    // No line processing:
    // echo off, echo newline off, canonical mode off,
    // extended input processing off, signal chars off
    config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    // Turn off character processing
    // clear current char size mask, no parity checking,
    // no output processing, force 8 bit input
    config.c_cflag &= ~(CSIZE | PARENB);
    config.c_cflag |= CS8;

    // One input byte is enough to return from read()
    // Inter-character timer off
    config.c_cc[VMIN]  = 1;
    config.c_cc[VTIME] = 10; // was 0

    // Get the current options for the port
    ////struct termios options;
    ////tcgetattr(fd, &options);

    // Apply baudrate
    switch (baud)
    {
        case 1200:
            if (cfsetispeed(&config, B1200) < 0 || cfsetospeed(&config, B1200) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 1800:
            cfsetispeed(&config, B1800);
            cfsetospeed(&config, B1800);
            break;
        case 9600:
            cfsetispeed(&config, B9600);
            cfsetospeed(&config, B9600);
            break;
        case 19200:
            cfsetispeed(&config, B19200);
            cfsetospeed(&config, B19200);
            break;
        case 38400:
            if (cfsetispeed(&config, B38400) < 0 || cfsetospeed(&config, B38400) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 57600:
            if (cfsetispeed(&config, B57600) < 0 || cfsetospeed(&config, B57600) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 115200:
            if (cfsetispeed(&config, B115200) < 0 || cfsetospeed(&config, B115200) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;

        // These two non-standard (by the 70'ties ) rates are fully supported on
        // current Debian and Mac OS versions (tested since 2010).
        case 460800:
            if (cfsetispeed(&config, B460800) < 0 || cfsetospeed(&config, B460800) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        case 921600:
            if (cfsetispeed(&config, B921600) < 0 || cfsetospeed(&config, B921600) < 0)
            {
                fprintf(stderr, "\nERROR: Could not set desired baud rate of %d Baud\n", baud);
                return false;
            }
            break;
        default:
            fprintf(stderr, "ERROR: Desired baud rate %d could not be set, aborting.\n", baud);
            return false;
    }

    // Finally, apply the configuration
    if(tcsetattr(fd, TCSAFLUSH, &config) < 0)
    {
        fprintf(stderr, "\nERROR: could not set configuration of fd %d\n", fd);
        return false;
    }

    // Done!
    return true;
}

// ------------------------------------------------------------------------------
//   Read Port with Lock
// ------------------------------------------------------------------------------
int
Serial_Port_Mav::
_read_port(char *buf, int sz)
{
    int result = -1;

    // Lock
    pthread_mutex_lock(&port_lock);
    result = read(fd, buf, sz);
    // Unlock
    pthread_mutex_unlock(&port_lock);

    return result;
}


// ------------------------------------------------------------------------------
//   Write Port with Lock
// ------------------------------------------------------------------------------
int
Serial_Port_Mav::
_write_port(char *buf, unsigned len)
{
    int bytesWritten = -1;

    // No need to lock port???
    // pthread_mutex_lock(&port_lock);
    // Write packet via serial link
    bytesWritten = static_cast<int>(write(fd, buf, len));
    // Wait until all data has been written
    tcdrain(fd);
    // Unlock
    // pthread_mutex_unlock(&port_lock);

    return bytesWritten;
}
