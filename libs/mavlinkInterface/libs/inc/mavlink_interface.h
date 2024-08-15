#ifndef MAVLINK_INTERFACE_H_
#define MAVLINK_INTERFACE_H_

#include "generic_port.h"

#include <stdio.h>
#include <iostream>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <pthread.h> // This uses POSIX Threads
#include <unistd.h>  // UNIX standard function definitions
#include <mutex>
#include <queue>
#include <string>

#include "user_mavlink_common.h"

/* Mavlink log.
   class: must be a pointer to class
 */
// #define MAV_DEBUG
#ifdef MAV_DEBUG
#define MAVLINK_LOG(fmt, ...)               \
    printf("MAVLINK[%s] %s(): " fmt "\n",   \
            get_comp_name(),                \
            __func__,                       \
            ##__VA_ARGS__);
#else
#define MAVLINK_LOG(fmt, ...)
    ;
#endif
    
#define MAVLINK_SUCCESS        0
#define MAVLINK_ERR_PORT_OPEN -1
#define MAVLINK_ERR_RECV_MSG  -2
#define MAVLINK_ERR_SEND_MSG  -3

/**
 * mavlink device infor struct
 **/
typedef struct _mavlink_dev_info {
    std::string comp_name;
    int system_id;
    int companion_id;
    int version;
    int channel;
    union {
        struct {
            const char *name;
            int baudrate;
        } uart_info;
        struct {
            const char *ip_addr;
            int port;
        } udp_info;
    } port;
} mavlink_dev_info_t;

// helper functions
void* mavlink_interface_read_thread(void *args);
void* mavlink_interface_write_thread(void *args);

// ----------------------------------------------------------------------------------
//   Time
// ------------------- ---------------------------------------------------------------
static uint64_t get_time_usec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

static uint64_t get_time_msec()
{
    static struct timeval _time_stamp;
    gettimeofday(&_time_stamp, NULL);
    return _time_stamp.tv_sec*1000 + _time_stamp.tv_usec;
}

class Mavlink_Interface
{
public:
    Mavlink_Interface(){};
    Mavlink_Interface(const mavlink_dev_info_t *mav_dev_info, bool use_serial=false);
    virtual ~Mavlink_Interface();

    void set_system_id(int sys_id);
    int get_system_id();
    void set_companion_id(int comp_id);
    int get_companion_id();
    int get_port_channel();
    int get_port_version();
    const char *get_comp_name();
    void quit_handler();

    int start();
    int stop();

    int8_t receive_message(mavlink_message_t &message, int8_t &q_size);
    int8_t send_message(mavlink_message_t message);
    virtual void w_heartbeat();

    void read_handler();
    void write_handler(void);
    bool does_children_exit();

    virtual int write_message(mavlink_message_t message);
    uint8_t msg_seq = 0;

    Generic_Port *port;

    void disableHeartbeatrSend();

private:
    
    bool time_to_exit;
    bool is_read_thread_run;
    bool is_write_thread_run;

    pthread_t read_tid;
    pthread_t write_tid;

    void read_thread();
    void write_thread(void);

    char reading_status;
    char writing_status;
    char control_status;
    uint64_t write_count;

    int system_id;
    int autopilot_id;
    int companion_id;
    std::string component_name;

    int read_messages();
    

    void handle_quit(int sig);

    // mavlink message queue
    pthread_mutex_t read_msg_q_mutex = PTHREAD_MUTEX_INITIALIZER;
    std::queue<mavlink_message_t> read_msg_q;

    pthread_mutex_t write_msg_q_mutex = PTHREAD_MUTEX_INITIALIZER;
    std::queue<mavlink_message_t> write_msg_q;

    bool is_enable_heartbeat = true;
};

#endif /* MAVLINK_INTERFACE_H_ */
