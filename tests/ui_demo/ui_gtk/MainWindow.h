#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <gtkmm.h>
#include <string>
#include <cstdio>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "PayloadSettingsTab.h"

typedef void (*ui_callback_t)(int event, double* param);
typedef void (*ui_connect_callback_t)(int event, const char* param);

class MainWindow : public Gtk::Window {
public:
    MainWindow(int width, int height);

    void regUICommandChanged(ui_callback_t func);
    void regUIConnectCommandChanged(ui_connect_callback_t func);
    ui_callback_t __notifyUICommandChanged;
    ui_connect_callback_t __notifyUIConnectCommandChanged;
    
    void send_connected();
    void send_disconnected();

    // Method to update information tab
    void update_storage_info(int status, double total, double used, double available);
    void update_capture_info(int img_status, int video_status, int img_count, int rec_time_ms);
    void update_url_streaming(char* url);

    void update_gimbal_attitude(float pitch, float roll, float yaw);
    void update_payload_status(double* params);
    void update_payload_param(char* index, double value);

private:
    bool is_connected = false;
    Gtk::Box main_box;
    Gtk::Box side_box;
    Gtk::Box tab_content;

    Gtk::Entry* ip_entry = nullptr;
    Gtk::Label* connect_info = nullptr;
    Gtk::Button* btn_connect = nullptr;
    
    // Tab instances
    PayloadSettingsTab* payload_tab = nullptr;

    Gtk::Widget* create_connect_ip();
    void on_payload_button_clicked(_index_notify index, double* param);
    void on_connect_button_clicked(_index_notify index, const char* param);

};


#endif