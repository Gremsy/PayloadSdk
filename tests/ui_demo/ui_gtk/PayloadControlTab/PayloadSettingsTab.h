#ifndef PAYLOADSETTINGSTAB_H
#define PAYLOADSETTINGSTAB_H

#include <gtkmm.h>
#include <string>
#include <cstdio>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "payloadSdkInterface.h"
#include "vio_sdk.h"

enum _index_notify{
    CAM_VIEW_MODE = 0,
    CAM_SOURCE_RECORD,
    CAM_CAPTURE,
    CAM_RECORD,
    CAM_EO_SPEED_ZOOM,
    CAM_ZOOM_CONTINIOUS,
    CAM_ZOOM_STEP,
    CAM_ZOOM_RANGE,
    CAM_EO_SPEED_FOCUS,
    CAM_FOCUS_CONTINIOUS,
    CAM_FOCUS_AUTO,
    CAM_AE_MODE,
    CAM_SHUTTER,
    CAM_IRIS,
    CAM_GAIN,
    CAM_WHITE_BALANCE,
    CAM_WHITE_BALANCE_TRIGGER,
    CAM_IR_PALETTE,
    CAM_LRF_MODE,
    CAM_OSD_MODE,
    CAM_IMAGE_FLIP,
    GIMBAL_CONTROL_TILT,
    GIMBAL_CONTROL_PAN,
    GIMBAL_CONTROL_ANGLE,
    GIMBAL_MODE,
    QUERY_PAYLOAD_PARAM
};

struct list_struct_t {
    std::string label;
    double value;

    list_struct_t() : label(""), value(0) {}

    list_struct_t(const std::string& vs, double v) : label(vs), value(v) {}
};

class PayloadSettingsTab : public Gtk::Box {
public:
    PayloadSettingsTab();

    sigc::signal<void, _index_notify, double*> signal_button_clicked();

    void update_storage_info(int status, double total, double used, double available);
    void update_capture_info(int img_status, int video_status, int img_count, int rec_time_ms);

    void query_payload_param();
    void update_gimbal_attitude(float pitch, float roll, float yaw);
    void update_payload_status(double* params);
    void update_payload_param(char* index, double value);


private:
    sigc::signal<void, _index_notify, double*> m_signal_button_clicked;


private:
    // Group creation methods
    Gtk::Widget* create_payload_setting_main_group();
    Gtk::Widget* create_camera_setting_main_group();
    Gtk::Widget* create_gimbal_setting_main_group();
    Gtk::Widget* create_info_show_main_group();

    // Group Camera Setting
    Gtk::Widget* create_capture_record_group();
    Gtk::Widget* create_zoom_controls_group();
    Gtk::Widget* create_focus_controls_group();
    Gtk::Widget* create_exposure_group();
    Gtk::Widget* create_white_balance_group();
    Gtk::Widget* create_ir_palette_group();
    Gtk::Widget* create_lrf_mode_group();
    Gtk::Widget* create_osd_mode_group();
    Gtk::Widget* create_image_flip_group();

    // Group Gimbal Setting
    Gtk::Widget* create_gimbal_control_speed_group();
    Gtk::Widget* create_gimbal_control_angle_group();

    Gtk::Widget* create_combo_box(Gtk::ComboBoxText*& combo, const std::string& title, const list_struct_t* buttons, int size, _index_notify index);
    void update_combo_by_value(Gtk::ComboBoxText* combo, const list_struct_t* list, int size, double value);

    Gtk::Widget* create_info_row(const std::string& title, Gtk::Label*& label);

    // Helper methods for zoom controls
    void create_range_eo_speed_zoom(Gtk::Box& parent_box);
    void create_continuous_zoom_controls(Gtk::Box& parent_box);
    void create_step_zoom_controls(Gtk::Box& parent_box);
    void create_range_zoom_controls(Gtk::Box& parent_box);

    void create_range_eo_speed_focus(Gtk::Box& parent_box);

    // General helper methods
    void add_button_to_box(Gtk::Box& box, const std::string& label, _index_notify index, double param);
    void bind_control_gimbal_button(Gtk::Button* button, _index_notify index, double value);

    // Event handlers
    void on_button_clicked(_index_notify index, double* value);
    void on_eo_zoom_speed_changed();
    void on_zoom_range_changed();
    void on_speed_gimbal_changed();
    void on_angle_gimbal_changed();
    void on_eo_focus_speed_changed();

private:
    Gtk::Scale* eo_zoom_speed_range = nullptr;
    Gtk::Scale* eo_focus_speed_range = nullptr;
    Gtk::Scale* zoom_range = nullptr;
    Gtk::Scale* speed_gimbal_range = nullptr;

    Gtk::Scale* pitch_angle_gimbal_range = nullptr;
    Gtk::Scale* roll_angle_gimbal_range  = nullptr;
    Gtk::Scale* yaw_angle_gimbal_range   = nullptr;

    Gtk::ComboBoxText* view_mode_combo   = nullptr;
    Gtk::ComboBoxText* rec_src_combo   = nullptr;
    Gtk::ComboBoxText* ae_mode_combo   = nullptr;
    Gtk::ComboBoxText* shutter_combo   = nullptr;
    Gtk::ComboBoxText* iris_combo   = nullptr;
    Gtk::ComboBoxText* gain_combo   = nullptr;
    Gtk::ComboBoxText* wb_mode_combo   = nullptr;
    Gtk::ComboBoxText* ir_palette_combo   = nullptr;
    Gtk::ComboBoxText* lrf_mode_combo   = nullptr;
    Gtk::ComboBoxText* osd_mode_combo   = nullptr;
    Gtk::ComboBoxText* image_flip_combo   = nullptr;
    Gtk::ComboBoxText* gimbal_mode_combo   = nullptr;

    Gtk::Label* storage_info = nullptr;
    Gtk::Label* capture_info = nullptr;
    Gtk::Label* record_info  = nullptr;

    Gtk::Label* gimbal_mode_info       = nullptr;
    Gtk::Label* pitch_angle_info       = nullptr;
    Gtk::Label* roll_angle_info        = nullptr;
    Gtk::Label* yaw_angle_info         = nullptr;
    Gtk::Label* view_mode_info         = nullptr;
    Gtk::Label* record_src_info        = nullptr;
    Gtk::Label* eo_zoom_level_info     = nullptr;
    Gtk::Label* ir_zoom_level_info     = nullptr;
    Gtk::Label* ir_type_info           = nullptr;
    Gtk::Label* ir_palette_info        = nullptr;
    Gtk::Label* ir_ffc_mode_info       = nullptr;
    Gtk::Label* ir_temp_max_info       = nullptr;
    Gtk::Label* ir_temp_min_info       = nullptr;
    Gtk::Label* ir_temp_mean_info      = nullptr;
    Gtk::Label* lrf_range_info         = nullptr;
    Gtk::Label* lrf_offset_x_info      = nullptr;
    Gtk::Label* lrf_offset_y_info      = nullptr;
    Gtk::Label* target_gps_lon_info    = nullptr;
    Gtk::Label* target_gps_lat_info    = nullptr;
    Gtk::Label* target_gps_alt_info    = nullptr;
    Gtk::Label* payload_gps_lon_info   = nullptr;
    Gtk::Label* payload_gps_lat_info   = nullptr;
    Gtk::Label* payload_gps_alt_info   = nullptr;

    list_struct_t cam_view_list[6] = {
        {"EO/IR",   0},
        {"EO",      1},
        {"IR",      2},
        {"IR/EO",   3},
        {"SYNC",    4},
        {"SBS",     6}
    };

    list_struct_t cam_src_list[4] = {
        {"EO",      1},
        {"IR",      2},
        {"BOTH",    0},
        {"OSD",     5}
    };

    list_struct_t gimbal_mode_list[4] = {
        {"OFF",      0},
        {"LOCK",     1},
        {"FOLLOW",   2},
        {"MAPPING",  3}
    };

    list_struct_t AE_mode_list[5] = {
        {"AUTO",     0},
        {"MANUAL",   3},
        {"SHUTTER",  10},
        {"IRIS",     11},
        {"GAIN",     14}
    };

    list_struct_t shutter_mode_list[28] = {
        {"1/1",     6},
        {"2/3",     7},
        {"1/2",     8},
        {"1/3",     9},
        {"1/4",     10},
        {"1/6",     11},
        {"1/8",     12},
        {"1/10",    13},
        {"1/15",    14},
        {"1/20",    15},
        {"1/30",    16},
        {"1/50",    17},
        {"1/60",    18},
        {"1/90",    19},
        {"1/100",   20},
        {"1/125",   21},
        {"1/180",   22},
        {"1/250",   23},
        {"1/350",   24},
        {"1/500",   25},
        {"1/725",   26},
        {"1/1000",  27},
        {"1/1500",  28},
        {"1/2000",  29},
        {"1/3000",  30},
        {"1/4000",  31},
        {"1/6000",  32},
        {"1/10000", 33}
    };

    list_struct_t iris_mode_list[15] = {
        {"F2.0",    25},
        {"F2.2",    24},
        {"F2.4",    23},
        {"F2.6",    22},
        {"F2.8",    21},
        {"F3.1",    20},
        {"F3.4",    19},
        {"F4.0",    17},
        {"F5.2",    14},
        {"F6.8",    11},
        {"F7.3",    10},
        {"F8.7",    8},
        {"F9.6",    7},
        {"F10",     6},
        {"F11",     5}
    };

    list_struct_t gain_mode_list[13] = {
        {"36 dB",    13},
        {"33 dB",    12},
        {"30 dB",    11},
        {"27 dB",    10},
        {"24 dB",    9},
        {"21 dB",    8},
        {"18 dB",    7},
        {"15 dB",    6},
        {"12 dB",    5},
        {" 9 dB",    4},
        {" 6 dB",    3},
        {" 3 dB",    2},
        {" 0 dB",    1}
    };

    list_struct_t white_balance_list[13] = {
        {"Auto",        0},
        {"Indoor",      1},
        {"Outdoor",     2},
        {"One Push WB", 3},
        {"ATW",         4},
        {"Manual",      5}
    };

    list_struct_t ir_palette_list[10] = {
        {"Palette 1",  0},
        {"Palette 2",  1},
        {"Palette 3",  2},
        {"Palette 4",  3},
        {"Palette 5",  4},
        {"Palette 6",  5},
        {"Palette 7",  6},
        {"Palette 8",  7},
        {"Palette 9",  8},
        {"Palette 10", 9}
    };

    list_struct_t lrf_mode_list[4] = {
        {"OFF",   3},
        {"1 Hz",  0},
        {"4 Hz",  1},
        {"10 Hz", 2}
    };

    list_struct_t osd_mode_list[4] = {
        {"Disable",  0},
        {"Debug",    1},
        {"Status",   2}
    };

    list_struct_t image_flip_list[4] = {
        {"OFF",  3},
        {"ON",   2}
    };


    std::map<Gtk::Button*, sigc::connection> m_button_timers;
    double speed_gimbal = 0.0;
    int rec_status = 0;

};

#endif