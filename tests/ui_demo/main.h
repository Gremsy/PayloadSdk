#ifndef MAIN_H_
#define MAIN_H_

#include <MainWindow.h>

MainWindow* window = nullptr;
void initPayloadControlUI();
void onUICommandChanged(int event, double* param);

#include"payloadSdkInterface.h"

T_ConnInfo s_conn = {
    CONTROL_UDP,
    udp_ip_target,
    udp_port_target
};

PayloadSdkInterface* my_payload = nullptr;
void initPayloadSDKInterface();
void onPayloadStatusChanged(int event, double* param);
void onPayloadParamChanged(int event, char* param_char, double* param);




#endif