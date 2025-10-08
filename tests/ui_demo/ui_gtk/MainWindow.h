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

class MainWindow : public Gtk::Window {
public:
    MainWindow(int width, int height);

    void regUICommandChanged(ui_callback_t func);
    ui_callback_t __notifyUICommandChanged;
    
    // Method to update information tab
    void update_storage_info(int status, double total, double used, double available);
    void update_capture_info(int img_status, int video_status, int img_count, int rec_time_ms);

    void update_gimbal_attitude(float pitch, float roll, float yaw);
    void update_payload_status(double* params);
    void update_payload_param(char* index, double value);

private:
    Gtk::Box main_box;
    Gtk::Box side_box;
    Gtk::Box tab_content;
    
    // Tab instances
    PayloadSettingsTab* payload_tab;

    void on_payload_button_clicked(_index_notify index, double* param);

};


#endif