#include "mavlink_interface.h"
#include "serial_port_mav.h"
#include "udp_port.h"

using namespace std;

/* Below definitions to solve issues when multiple mavlink
   connections are using.
 */
mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
static Generic_Port *port_quit;

// ----------------------------------------------------------------------------------
//   Mavlink Interface Class
// ----------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
//   Pulic
// ------------------------------------------------------------------------------
Mavlink_Interface::
Mavlink_Interface(const mavlink_dev_info_t *mav_dev_info, bool use_serial)
{
    Generic_Port *newport;

    if(use_serial)
        newport = new Serial_Port_Mav(mav_dev_info->port.uart_info.name,
                                  mav_dev_info->port.uart_info.baudrate);
    else
        newport = new UDP_Port(mav_dev_info->port.udp_info.ip_addr,
                               mav_dev_info->port.udp_info.port);
    
    if (!newport) {
        MAVLINK_LOG("Faield to create new port!");
        exit(MAVLINK_ERR_PORT_OPEN);
    }

    port = newport;
    port_quit = newport;

    // initialize attributes
    write_count = 0;
    reading_status = 0;      // whether the read thread is running
    writing_status = 0;      // whether the write thread is running
    control_status = 0;      // whether the autopilot is in offboard control mode
    time_to_exit   = false;  // flag to signal thread exit
    is_write_thread_run = false;
    is_read_thread_run = false;

    read_tid  = 0; // read thread id
    write_tid = 0; // write thread id

    system_id    = mav_dev_info->system_id; // system id
    autopilot_id = 25; // autopilot component id
    companion_id = mav_dev_info->companion_id; // companion computer component id
    component_name = mav_dev_info->comp_name;

    port->set_mavlink_channel(mav_dev_info->channel);
    port->set_mavlink_version(mav_dev_info->version);
    mavlink_set_proto_version(mav_dev_info->channel, mav_dev_info->version);

    MAVLINK_LOG("Mavlink init successfully!!!");
}

Mavlink_Interface::
~Mavlink_Interface() {
    delete port;
}

void
Mavlink_Interface::
set_system_id(int sys_id) {
    system_id = sys_id;
}

int
Mavlink_Interface::
get_system_id() {
    return system_id;
}

void
Mavlink_Interface::
set_companion_id(int comp_id) {
    companion_id = comp_id;
}

int
Mavlink_Interface::
get_companion_id() {
    return companion_id;
}

int
Mavlink_Interface::
get_port_channel() {
    return port->get_mavlink_channel();
}

int
Mavlink_Interface::
get_port_version() {
    return port->get_mavlink_version();
}

bool
Mavlink_Interface::
does_children_exit() {
    return time_to_exit;
}

// User must call this in SIGINT (Ctrl-C) handler function.
void
Mavlink_Interface::
quit_handler()
{
    MAVLINK_LOG("\nTerminating\n");

    // autopilot interface
    try {
        stop();
    }
    catch (int error){
        MAVLINK_LOG("Couldn't stop threads (%d)", error);
    }

    // port
    try {
        port_quit->stop();
    }
    catch (int error){
        MAVLINK_LOG("Couldn't stop port (%d)", error);
    }
}

int8_t
Mavlink_Interface::
receive_message(mavlink_message_t &message, int8_t &q_size) {
    int ret = MAVLINK_ERR_RECV_MSG;

    pthread_mutex_lock(&read_msg_q_mutex);
    if (!read_msg_q.empty()) {
        // MAVLINK_LOG("queue sz: %d", read_msg_q.size());
        q_size = read_msg_q.size();
        message = read_msg_q.front();
        read_msg_q.pop();
        ret = MAVLINK_SUCCESS;
    }
    pthread_mutex_unlock(&read_msg_q_mutex);

    return ret;
}

int8_t
Mavlink_Interface::
send_message(mavlink_message_t message) {
    int ret = MAVLINK_ERR_SEND_MSG;

    pthread_mutex_lock(&write_msg_q_mutex);
    // MAVLINK_LOG("queue sz: %d", write_msg_q.size());
    write_msg_q.push(message);
    pthread_mutex_unlock(&write_msg_q_mutex);

    ret = MAVLINK_SUCCESS;

    return ret;
}

const char *
Mavlink_Interface::
get_comp_name() {
    return component_name.c_str();
}

// ------------------------------------------------------------------------------
//   STARTUP
// ------------------------------------------------------------------------------
int
Mavlink_Interface::
start()
{
    int ret = -1;

    // --------------------------------------------------------------------------
    //   Setup PORT and verify
    // --------------------------------------------------------------------------
    MAVLINK_LOG("Start port");
    port->start();
    if (!port->is_running()) // PORT_OPEN
    {
        MAVLINK_LOG("ERROR: port not open");
        return -1;
    }

    // --------------------------------------------------------------------------
    //   READ THREAD
    // --------------------------------------------------------------------------
    MAVLINK_LOG("Start read thread");
    ret = pthread_create( &read_tid, NULL, &mavlink_interface_read_thread, this);
    if (ret) {
        MAVLINK_LOG("Failed to create read thread!");
        return ret;
    }

    // --------------------------------------------------------------------------
    //   WRITE THREAD
    // --------------------------------------------------------------------------
    MAVLINK_LOG("Start write thread");
    ret = pthread_create( &write_tid, NULL, &mavlink_interface_write_thread, this);
    if (ret) {
        MAVLINK_LOG("Failed to create write thread!");
        return ret;
    }

    // wait for it to be started
    while (not writing_status)
        usleep(100000); // 10Hz

    MAVLINK_LOG("Start successfully!");
    // Done!
    return ret;
}

// ------------------------------------------------------------------------------
//   SHUTDOWN
// ------------------------------------------------------------------------------
int
Mavlink_Interface::
stop()
{
    // --------------------------------------------------------------------------
    //   CLOSE THREADS
    // --------------------------------------------------------------------------
    MAVLINK_LOG("Close threads");

    // signal exit
    time_to_exit = true;

    // wait for exit
    pthread_join(read_tid, NULL);
    pthread_join(write_tid, NULL);

    // still need to close the port separately
    return 0;
}

// ------------------------------------------------------------------------------
//   Private
// ------------------------------------------------------------------------------
int
Mavlink_Interface::
read_messages() {
    int ret = -1;

    if (time_to_exit)
        return ret;

    queue<mavlink_message_t> mavlink_message_q;
    ret = port->read_message(mavlink_message_q);
    if (ret <= 0) {
        // MAVLINK_LOG("Failed to read messages from port!")
        return ret;
    }

    pthread_mutex_lock(&read_msg_q_mutex);
    while (!mavlink_message_q.empty()) {
        read_msg_q.push(mavlink_message_q.front());
        mavlink_message_q.pop();
    }
    pthread_mutex_unlock(&read_msg_q_mutex);

    // give the write thread time to use the port
    if (writing_status > 0)
        usleep(100); // look for components of batches at 10kHz

    return ret;
}

int
Mavlink_Interface::
write_message(mavlink_message_t message)
{
    // message.seq = msg_seq++;
    
    // do the write
    int len = port->write_message(message);

    // book keep
    ++write_count;

    // Done!
    return len;
}

// ------------------------------------------------------------------------------
//   Read Thread
// ------------------------------------------------------------------------------
void
Mavlink_Interface::
read_thread()
{
    reading_status = true;
    int err_cnt = 0;
    int reset = 0;

    while (!time_to_exit)
    {
        int ret = read_messages();
        if (ret < 0) {
            ++err_cnt;
            if (err_cnt >= 5) {
                is_read_thread_run = false;
                if (is_write_thread_run == false)
                    time_to_exit = true;
                MAVLINK_LOG("Read thread exit!");
                pthread_exit(NULL);
            }
        }
        ++reset;
        /* If less than 5 errors in 5', reset error count */
        if (reset >= 1500000) {
            err_cnt = 0;
            reset = 0;
        }
        usleep(200); // Read batches at 500Hz
    }

    reading_status = false;

    return;
}

void
Mavlink_Interface::
read_handler()
{
    if (reading_status != 0)
    {
        MAVLINK_LOG("Read thread already running");
        return;
    }
    else
    {
        is_read_thread_run = true;
        read_thread();
        return;
    }
}

// ------------------------------------------------------------------------------
//   Write Thread
// ------------------------------------------------------------------------------
void
Mavlink_Interface::
write_thread(void)
{
    int err_cnt = 0;
    int reset = 0;
    int cnt = 0;
    int tick = 0;
    int len;
    mavlink_message_t message;
    bool have_message = false;

    writing_status = true;
    while (!time_to_exit)
    {
        cnt++;
        /* Send hearbeat once every second */
        if(cnt >= 1000) {
            cnt = 0;
            w_heartbeat();
            MAVLINK_LOG("Write hb %d", tick++);
        } else {

            pthread_mutex_lock(&write_msg_q_mutex);
            if(write_msg_q.size() > 0){
                // MAVLINK_LOG("queue sz: %d", write_msg_q.size());
                message = write_msg_q.front();
                write_msg_q.pop();
                have_message = true;
            }
            pthread_mutex_unlock(&write_msg_q_mutex);

            if (have_message == true) {
                len = write_message(message);
                if ( len <= 0 ) {
                    MAVLINK_LOG("Failed to write messages to port!");
                    ++err_cnt;
                    if (err_cnt >= 5) {
                        is_write_thread_run = false;
                        if (is_read_thread_run == false)
                            time_to_exit = true;
                        MAVLINK_LOG("Write thread exit!");
                        pthread_exit(NULL);
                    }
                }

                have_message = false;
                // MAVLINK_LOG("Mavlink write queue size %d", write_msg_q.size());
                // MAVLINK_LOG("Write cnt %d", tick++);
            }
        }
        ++reset;
        /* If less than 5 errors in 5', reset error count */
        if (reset >= 30000) {
            err_cnt = 0;
            reset = 0;
        }
        usleep(1000);
    }

    // signal end
    writing_status = false;

    return;
}

void
Mavlink_Interface::
write_handler(void)
{
    if (not writing_status == false)
    {
        MAVLINK_LOG("write thread already running");
        return;
    }
    else
    {
        is_write_thread_run = true;
        write_thread();
        return;
    }
}

// ------------------------------------------------------------------------------
//  Pthread Starter Helper Functions
// ------------------------------------------------------------------------------

void*
mavlink_interface_read_thread(void *args)
{
    // takes an mavlink object argument
    Mavlink_Interface *mav_intf = (Mavlink_Interface *)args;

    // run the object's read thread
    mav_intf->read_handler();

    // done!
    return NULL;
}

void*
mavlink_interface_write_thread(void *args)
{
    // takes an mavlink object argument
    Mavlink_Interface *mav_intf = (Mavlink_Interface *)args;

    // run the object's read thread
    mav_intf->write_handler();

    // done!
    return NULL;
}

void 
Mavlink_Interface::
w_heartbeat(){
    if(!is_enable_heartbeat) return;
    
    mavlink_heartbeat_t heartbeat = { 0 };
    heartbeat.type          = MAV_TYPE_GENERIC;
    heartbeat.autopilot     = MAV_AUTOPILOT_INVALID;
    heartbeat.base_mode     = 0;
    heartbeat.custom_mode   = 0;
    heartbeat.system_status = MAV_STATE_ACTIVE;
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    mavlink_message_t message = { 0 };
    mavlink_msg_heartbeat_encode_chan(get_system_id(), get_companion_id(), get_port_channel(), &message, &heartbeat);

    // --------------------------------------------------------------------------
    //   WRITE
    // --------------------------------------------------------------------------
    if (send_message(message) < 0)
        fprintf(stderr, "WARNING: could not send HEARTBEAT\n");
}

void 
Mavlink_Interface::
disableHeartbeatrSend(){
    is_enable_heartbeat = false;
}