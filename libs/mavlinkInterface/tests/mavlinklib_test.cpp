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
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlinklib_test.h"

#define SYS_ID              240
#define COMP_ID             MAV_COMP_ID_SYSTEM_CONTROL
#define MAV_VERSION         2
#define MAV_CHANNEL         MAVLINK_COMM_1

#define PAYLOAD_DEVICE      O_PAYLOAD

/* Cam information */
struct CameraInfo {
    uint8_t vendorName[32];          /**< Name of the camera device vendor. */
    uint8_t modelName[32];           /**< Name of the camera device model. */
    uint32_t firmware_version;       /**< Firmware version. */
    float focal_length;              /**< Focal length. */
    float sensor_size_h;             /**< Sensor size horizontol. */
    float sensor_size_v;             /**< Sensor size vertical. */
    uint16_t resolution_h;           /**< Resolution horizontol. */
    uint16_t resolution_v;           /**< Resolution vertical. */
    uint8_t lens_id;                 /**< Lens ID. */
    uint32_t flags;                  /**< Flags */
    uint16_t cam_definition_version; /**< Camera Definition file version */
    char cam_definition_uri[140];    /**< Camera Definition file URI address */

    CameraInfo(){
        #if (PAYLOAD_DEVICE == ZIO_PAYLOAD)
            payload_zio();
        #elif (PAYLOAD_DEVICE == DUO_PAYLOAD)
            payload_duo();
        #elif (PAYLOAD_DEVICE == O_PAYLOAD)
            payload_o();
        #endif
    }

#if (PAYLOAD_DEVICE == ZIO_PAYLOAD)
    void payload_zio(){
        strcpy((char *)vendorName, "Gremsy JSC");
        strcpy((char *)modelName, "Zio Payload");
        firmware_version = 0x1000000;
        focal_length = 0;
        sensor_size_h = 23.8;
        sensor_size_v = 37.5;
        resolution_h = 1920;
        resolution_v = 1080;
        lens_id = 1;

        flags = (CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM) | (CAMERA_CAP_FLAGS_HAS_MODES) | (CAMERA_CAP_FLAGS_CAPTURE_IMAGE) | (CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
        cam_definition_version = 1;
        strcpy((char *)cam_definition_uri, "https://github.com/Gremsy/ZIO_Camera_Definition/releases/download/v4.0.0/camera_def_zio.xml");
    }

#elif (PAYLOAD_DEVICE == DUO_PAYLOAD)

    void payload_duo(){
        strcpy((char *)vendorName, "Gremsy JSC");
        strcpy((char *)modelName, "Duo Payload");
        firmware_version = 0x1000000;
        focal_length = 0;
        sensor_size_h = 23.8;
        sensor_size_v = 37.5;
        resolution_h = 3840;
        resolution_v = 2160;
        lens_id = 1;

        flags = (CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM) | (CAMERA_CAP_FLAGS_HAS_MODES) | (CAMERA_CAP_FLAGS_CAPTURE_IMAGE) | (CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
        cam_definition_version = 3;
        strcpy((char *)cam_definition_uri, "https://github.com/minhvogremsy/file_xml/releases/download/v1.0.0/camera_def_duo_edit.xml");
    }

#elif (PAYLOAD_DEVICE == O_PAYLOAD)

    void payload_o(){
        strcpy((char *)vendorName, "Gremsy JSC");
        strcpy((char *)modelName, "O Payload");
        firmware_version = 0x1000000;
        focal_length = 0;
        sensor_size_h = 23.8;
        sensor_size_v = 37.5;
        resolution_h = 3840;
        resolution_v = 2160;
        lens_id = 1;

        flags = (CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM) | (CAMERA_CAP_FLAGS_HAS_MODES) | (CAMERA_CAP_FLAGS_CAPTURE_IMAGE) | (CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
        cam_definition_version = 4;
        strcpy((char *)cam_definition_uri, "https://github.com/Gremsy/ZIO_Camera_Definition/releases/download/v4.0.0/camera_def_zio.xml");
    }
#endif /* PAYLOAD_DEVICE */
};

struct StorageInfo {
    uint8_t storage_id;
    uint8_t storage_count;
    uint8_t status;
    float total_capacity;
    float used_capacity;
    float available_capacity;
    float read_speed;
    float write_speed;

    StorageInfo(){
        defaultValue();
    }
    void defaultValue(){
        storage_id = 0;
        storage_count = 1;
        status = STORAGE_STATUS_EMPTY;
        total_capacity = 0;
        used_capacity = 0;
        available_capacity = 0;
        read_speed = 200;
        write_speed = 90;
    }
};

const struct CameraInfo camInfo;
const struct StorageInfo storageInfo;
pthread_t mav_tid = 0;

// Mavlinklib_Test::
// Mavlinklib_Test(std::string comp_name, const char *uart_name_ ,
//                       int baudrate_, int sys_id, int comp_id,
//                       int version, int channel) {
//     // Do nothing
// }

void*
mavlink_msg_process_thread(void *args)
{
    // takes an mavlink object argument
    Mavlinklib_Test *mav_test = (Mavlinklib_Test *)args;

    // run the object's read thread
    mav_test->msg_process();

    // done!
    return NULL;
}

void
Mavlinklib_Test::
w_heartbeat(){
    mavlink_heartbeat_t heartbeat;

    heartbeat.type          = MAV_TYPE_ONBOARD_CONTROLLER;
    heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode     = 0;
    heartbeat.custom_mode   = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;

    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message;

    mavlink_msg_heartbeat_encode_chan(get_system_id(),
                                      get_companion_id(),
                                      get_port_channel(),
                                      &message,
                                      &heartbeat);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    send_message(message);
    MAVLINK_LOG("Send heartbeat from sys %d comp %d chan %d",
                get_system_id(), get_companion_id(), get_port_channel());
}

Mavlinklib_Test::
~Mavlinklib_Test() {};

void
Mavlinklib_Test::
msg_process() {
    int hb_cnt = 0;

    while (1) {
        mavlink_message_t message_in;
        mavlink_message_t message_out;
        int8_t q_size = 0;
        int ret = receive_message(message_in, q_size);

        if (ret != MAVLINK_SUCCESS) {
            usleep(1000);
            if (does_children_exit()) {
                MAVLINK_LOG("Goodbye from %d!", (int)pthread_self());
                return;
            }
            continue;
        }

        switch (message_in.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT: {
                mavlink_heartbeat_t heartbeat{0};
                mavlink_msg_heartbeat_decode(&message_in, &(heartbeat));
                ++hb_cnt;
                MAVLINK_LOG("   ---> Heartbeat received from FC sysid: %d, copmid: %d",
                    message_in.sysid, message_in.compid);
                break;
            }
            case MAVLINK_MSG_ID_COMMAND_LONG:{
                mavlink_command_long_t command_long{0};
                uint16_t command;

                mavlink_msg_command_long_decode(&message_in, &(command_long));
                command = command_long.command;

                switch (command) {
                    case MAV_CMD_REQUEST_CAMERA_INFORMATION: {
                        MAVLINK_LOG("MAV_CMD_REQUEST_CAMERA_INFORMATION");
                        handle_request_camera_information(message_in, command_long.command);
                        usleep(100000);
                        break;
                    }
                    case MAV_CMD_REQUEST_STORAGE_INFORMATION: {
                        MAVLINK_LOG("MAV_CMD_REQUEST_STORAGE_INFORMATION");
                        handle_request_storage_information(message_in, command_long.command);
                        usleep(100000);
                        break;
                    }
                    case MAV_CMD_REQUEST_CAMERA_SETTINGS: {
                        MAVLINK_LOG("MAV_CMD_REQUEST_CAMERA_SETTINGS");
                        handle_request_camera_settings(message_in, command_long.command);
                        break;
                    }
                    default: {
                        break;
                    }
                }
            }
            case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:{
                MAVLINK_LOG("   ---> MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ");
                break;
            }
            default: {
                break;
            }
        }
    }
}

void
Mavlinklib_Test::
send_ack(mavlink_message_t msg_in, int cmd, bool success)
{
    mavlink_message_t msg_out;

    mavlink_msg_command_ack_pack(
        get_system_id() /*system_id*/, get_companion_id() /*component_id*/, &msg_out /*msg*/, cmd /*command*/,
        success /*result*/ ? MAV_RESULT_ACCEPTED : MAV_RESULT_FAILED, 0 /*progress*/,
        0 /*result_param2*/, msg_in.sysid /*target_system*/, msg_in.compid /*target_component*/);

    send_message(msg_out);
}

void
Mavlinklib_Test::
handle_request_camera_information(mavlink_message_t msg_in, uint16_t command) {
    mavlink_message_t msg_out;

    MAVLINK_LOG("Request from sys %d comp %d", msg_in.sysid, msg_in.compid);
    mavlink_msg_camera_information_pack(
        get_system_id(), get_companion_id(), &msg_out, 0, (const uint8_t *)camInfo.vendorName,
        (const uint8_t *)camInfo.modelName, camInfo.firmware_version, camInfo.focal_length,
        camInfo.sensor_size_h, camInfo.sensor_size_v, camInfo.resolution_h,
        camInfo.resolution_v, camInfo.lens_id, camInfo.flags, camInfo.cam_definition_version,
        (const char *)camInfo.cam_definition_uri);

    send_message(msg_out);
    send_ack(msg_in, command, true);
}

void
Mavlinklib_Test::
handle_request_storage_information(mavlink_message_t msg_in, uint16_t command) {
    mavlink_message_t msg_out;

    MAVLINK_LOG("Request from sys %d comp %d", msg_in.sysid, msg_in.compid);
    mavlink_msg_storage_information_pack(get_system_id(), get_companion_id(), &msg_out, 0,
                                         storageInfo.storage_id, storageInfo.storage_count,
                                         storageInfo.status, storageInfo.total_capacity,
                                         storageInfo.used_capacity, storageInfo.available_capacity,
                                         storageInfo.read_speed, storageInfo.write_speed,
                                         STORAGE_TYPE_MICROSD, "");
    send_message(msg_out);
    send_ack(msg_in, command, true);
}

void
Mavlinklib_Test::
handle_request_camera_settings(mavlink_message_t msg_in, uint16_t command) {
    mavlink_message_t msg_out;

    MAVLINK_LOG("Request from sys %d comp %d", msg_in.sysid, msg_in.compid);
    mavlink_msg_camera_settings_pack(get_system_id(), get_companion_id(), &msg_out, 0,
                                     CAMERA_MODE_IMAGE, NAN, NAN); // zoom level is unknown
    send_message(msg_out);
    send_ack(msg_in, command, true);
}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

    // string for command line usage
    const char *commandline_usage = "usage: mavlinklib_test [-d <devicename> -b <baudrate>]";

    // Read input arguments
    for (int i = 1; i < argc; i++) { // argv[0] is "mavlinklib_test"

        // Help
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            printf("%s\n", commandline_usage);
            throw EXIT_FAILURE;
        }

        // UART device ID
        if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
            if (argc > i + 1) {
                i++;
                uart_name = argv[i];
            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }

        // Baud rate
        if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
            if (argc > i + 1) {
                i++;
                baudrate = atoi(argv[i]);
            } else {
                printf("%s\n", commandline_usage);
                throw EXIT_FAILURE;
            }
        }
    }
    // end: for each input argument

    // Done!
    return;
}


// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
user_quit_handler( int sig )
{
    printf("\n");
    printf("\nTERMINATING AT USER REQUEST\n");
    printf("\n");

    // mavlink interface quit
    try {
        mav_test_quit->quit_handler();
    }
    catch (int error){}

    // MAVLINK_LOG(*mav_test_quit, "TID: %d", (int)mav_tid);
    pthread_join(mav_tid, NULL);
    // end program here
    exit(0);
}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
int
main(int argc, char **argv)
{
    // Default input arguments
#ifdef __APPLE__
    char *uart_name = (char*);
#else
    char *uart_name = (char*)"/dev/ttyUSB0";
#endif
    int baudrate = 115200;
    int ret = -1;

    /* Currently the Mavlink_Interface only support UART port */
    string component_name = "MavlinkLib-Test";

    // do the parse, will throw an int if it fails
    parse_commandline(argc, argv, uart_name, baudrate);

    // Init variables
    mavlink_dev_info_t mavlink_dev_info[1] = {
        {
            .comp_name = "MavTest",
            .system_id = SYS_ID,
            .companion_id = COMP_ID,
            .version = MAV_VERSION,
            .channel = MAV_CHANNEL,
            .port = {
                .uart_info = {
                    uart_name,
                    baudrate
                }
            }
        },
    };

    // --------------------------------------------------------------------------
    //   STARTUP
    // --------------------------------------------------------------------------

    /*
     * Instantiate an autopilot interface object
     *
     * This starts two threads for read and write over MAVlink. The read thread
     * listens for any MAVlink message and pushes it to the current_messages
     * attribute.  The write thread at the moment only streams a position target
     * in the local NED frame (mavlink_set_position_target_local_ned_t), which
     * is changed by using the method update_setpoint().  Sending these messages
     * are only half the requirement to get response from the autopilot, a signal
     * to enter "offboard_control" mode is sent by using the enable_offboard_control()
     * method.  Signal the exit of this mode with disable_offboard_control().  It's
     * important that one way or another this program signals offboard mode exit,
     * otherwise the vehicle will go into failsafe.
     *
     */
    Mavlinklib_Test mav_test(mavlink_dev_info);

    /*
     * Setup interrupt signal handler
     *
     * Responds to early exits signaled with Ctrl-C.  The handler will command
     * to exit offboard mode if required, and close threads and the port.
     * The handler in this example needs references to the above objects.
     *
     */
    mav_test_quit = &mav_test;
    signal(SIGINT, user_quit_handler);

    /*
     * Start mavlink_interface
     * This is where the port is opened, and read and write threads are started.
     */
    mav_test.start();

    // --------------------------------------------------------------------------
    //   Create thread for processing Mavlink messages
    // --------------------------------------------------------------------------
    ret = pthread_create( &mav_tid, NULL, &mavlink_msg_process_thread, &mav_test);
    if (ret < 0) {
        printf("Failed to create thread for process mavlink message!\n");
        return ret;
    }
    // MAVLINK_LOG(mav_test, "TID: %d", (int)mav_tid);

    while (1) {
        sleep(1);
    };

    /*
     * Now we can implement the operations with mavlink interface
     */
    // --------------------------------------------------------------------------
    //   THREAD and PORT SHUTDOWN
    // --------------------------------------------------------------------------

    /*
     * Now that we are done we can stop the threads and close the port
     */
    mav_test.stop();
    pthread_join(mav_tid, NULL);

    // --------------------------------------------------------------------------
    //   DONE
    // --------------------------------------------------------------------------

    printf("Exit mavlinklib test!\n");

    // woot!
    return 0;
}
