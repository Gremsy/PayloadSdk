#include "PayloadSettingsTab.h"


PayloadSettingsTab::PayloadSettingsTab() : Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 10) {
    set_margin_top(10);
    set_margin_bottom(10);
    set_margin_start(10);
    set_margin_end(10);

    // Create main groups
    pack_start(*create_payload_setting_main_group(), Gtk::PACK_SHRINK);
    pack_start(*create_camera_setting_main_group(), Gtk::PACK_SHRINK);
    pack_start(*create_gimbal_setting_main_group(), Gtk::PACK_SHRINK);
    pack_start(*create_info_show_main_group(), Gtk::PACK_SHRINK);

    // query_payload_param();
    Glib::signal_idle().connect_once([this]() {
        query_payload_param();
    });

}

// ===== Payload Setting Main Group =====
Gtk::Widget* 
PayloadSettingsTab::
create_payload_setting_main_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Payload Setting");
    this->signal_size_allocate().connect([frame](Gtk::Allocation& alloc){
        int parent_width = alloc.get_width();
        frame->set_size_request(parent_width * 0.20, 50);
    });
    frame->set_halign(Gtk::ALIGN_END);
    frame->set_valign(Gtk::ALIGN_START);

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 10);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);
    
    auto cam_frame = Gtk::make_managed<Gtk::Frame>();

    auto hbox = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 10);
    hbox->set_margin_top(10);
    hbox->set_margin_bottom(10);
    hbox->set_margin_start(10);
    hbox->set_margin_end(10);

    hbox->pack_start(*create_combo_box(view_mode_combo, "View", cam_view_list, sizeof(cam_view_list)/sizeof(cam_view_list[0]), CAM_VIEW_MODE), Gtk::PACK_EXPAND_WIDGET);
    hbox->pack_start(*create_combo_box(rec_src_combo, "Record", cam_src_list, sizeof(cam_src_list)/sizeof(cam_src_list[0]), CAM_SOURCE_RECORD), Gtk::PACK_EXPAND_WIDGET);

    cam_frame->add(*hbox);

    box->pack_start(*cam_frame, Gtk::PACK_SHRINK);

    box->pack_start(*create_capture_record_group(), Gtk::PACK_SHRINK);
    box->pack_start(*create_lrf_mode_group(), Gtk::PACK_SHRINK);
    box->pack_start(*create_osd_mode_group(), Gtk::PACK_SHRINK);
    box->pack_start(*create_image_flip_group(), Gtk::PACK_SHRINK);

    frame->add(*box);
    return frame;
}

// ===== Camera Setting Main Group =====
Gtk::Widget* 
PayloadSettingsTab::
create_camera_setting_main_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Camera Setting");
    this->signal_size_allocate().connect([frame](Gtk::Allocation& alloc){
        int parent_width = alloc.get_width();
        frame->set_size_request(parent_width * 0.20, -1);
    });
    frame->set_halign(Gtk::ALIGN_START);
    frame->set_valign(Gtk::ALIGN_START);
    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 10);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    box->pack_start(*create_zoom_controls_group(), Gtk::PACK_SHRINK);
    box->pack_start(*create_focus_controls_group(), Gtk::PACK_SHRINK);
    box->pack_start(*create_exposure_group(), Gtk::PACK_SHRINK);
    box->pack_start(*create_white_balance_group(), Gtk::PACK_SHRINK);
    box->pack_start(*create_ir_palette_group(), Gtk::PACK_SHRINK);

    frame->add(*box);
    return frame;
}

// ===== Gimbal Setting Main Group =====
Gtk::Widget* 
PayloadSettingsTab::
create_gimbal_setting_main_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Gimbal Setting");
    this->signal_size_allocate().connect([frame](Gtk::Allocation& alloc){
        int parent_width = alloc.get_width();
        frame->set_size_request(parent_width * 0.20, 50);
    });
    frame->set_halign(Gtk::ALIGN_START);
    frame->set_valign(Gtk::ALIGN_START);
    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 10);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    box->pack_start(*create_combo_box(gimbal_mode_combo, "Gimbal Mode", gimbal_mode_list, sizeof(gimbal_mode_list)/sizeof(gimbal_mode_list[0]), GIMBAL_MODE), Gtk::PACK_SHRINK);
    box->pack_start(*create_gimbal_control_speed_group(), Gtk::PACK_SHRINK);
    box->pack_start(*create_gimbal_control_angle_group(), Gtk::PACK_SHRINK);

    frame->add(*box);
    return frame;
}

// ===== Info Main Group =====
Gtk::Widget* 
PayloadSettingsTab::
create_info_show_main_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Payload Info");
    this->signal_size_allocate().connect([frame](Gtk::Allocation& alloc){
        int parent_width = alloc.get_width();
        frame->set_size_request(parent_width * 0.20, 50);
    });
    frame->set_halign(Gtk::ALIGN_END);
    frame->set_valign(Gtk::ALIGN_START);

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 10);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    box->pack_start(*create_info_row("Gimbal Mode", gimbal_mode_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("Pitch", pitch_angle_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("Roll", roll_angle_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("Yaw", yaw_angle_info), Gtk::PACK_SHRINK);

    box->pack_start(*create_info_row("View Mode", view_mode_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("Record Source", record_src_info), Gtk::PACK_SHRINK);

    box->pack_start(*create_info_row("EO Zoom Level", eo_zoom_level_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("IR Zoom Level", ir_zoom_level_info), Gtk::PACK_SHRINK);

    box->pack_start(*create_info_row("IR Type", ir_type_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("IR Palettte ID", ir_palette_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("IR FFC Mode", ir_ffc_mode_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("IR Temp Max", ir_temp_max_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("IR Temp Min", ir_temp_min_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("IR Temp Mean", ir_temp_mean_info), Gtk::PACK_SHRINK);

    box->pack_start(*create_info_row("LRF OFFSET X", lrf_offset_x_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("LRF OFFSET Y", lrf_offset_y_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("LRF Range", lrf_range_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("Target GPS LON", target_gps_lon_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("Target GPS LAT", target_gps_lat_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("Target GPS ALT", target_gps_alt_info), Gtk::PACK_SHRINK);

    box->pack_start(*create_info_row("Payload GPS LON", payload_gps_lon_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("Payload GPS LAT", payload_gps_lat_info), Gtk::PACK_SHRINK);
    box->pack_start(*create_info_row("Payload GPS ALT", payload_gps_alt_info), Gtk::PACK_SHRINK);

    frame->add(*box);
    return frame;
}

// ===== Capture/Record Group =====
Gtk::Widget* 
PayloadSettingsTab::
create_capture_record_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Capture / Record");

    auto main_box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 10);
    main_box->set_margin_top(0);
    main_box->set_margin_bottom(10);
    main_box->set_margin_start(10);
    main_box->set_margin_end(10);

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 10);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    auto capture_group = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 0);

    auto btn_capture = Gtk::make_managed<Gtk::Button>("Capture");
    btn_capture->signal_clicked().connect([this]() {
        double params[1] = {0.0};
        on_button_clicked(CAM_CAPTURE, params);
    });
    capture_group->pack_start(*btn_capture, Gtk::PACK_EXPAND_WIDGET);
    capture_info = Gtk::make_managed<Gtk::Label>("N/A");
    capture_info->set_markup("0");
    capture_group->pack_start(*capture_info, Gtk::PACK_EXPAND_WIDGET);

    auto record_group = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 0);
    auto btn_record  = Gtk::make_managed<Gtk::Button>("Record");
    btn_record->signal_clicked().connect([this]() {
        double params[1] = {(double)rec_status};
        on_button_clicked(CAM_RECORD, params);
    });
    record_group->pack_start(*btn_record, Gtk::PACK_EXPAND_WIDGET);
    record_info = Gtk::make_managed<Gtk::Label>("N/A");
    record_info->set_markup("00:00:00");
    record_group->pack_start(*record_info, Gtk::PACK_EXPAND_WIDGET);

    box->pack_start(*capture_group, Gtk::PACK_EXPAND_WIDGET);
    box->pack_start(*record_group, Gtk::PACK_EXPAND_WIDGET);

    main_box->pack_start(*box, Gtk::PACK_EXPAND_WIDGET);

    storage_info = Gtk::make_managed<Gtk::Label>("N/A");
    storage_info->set_markup("No SD Card");
    main_box->pack_start(*storage_info, Gtk::PACK_EXPAND_WIDGET);

    frame->add(*main_box);
    return frame;
}

Gtk::Widget*
PayloadSettingsTab::
create_zoom_controls_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Zoom");

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    create_range_eo_speed_zoom(*box);
    create_continuous_zoom_controls(*box);
    create_step_zoom_controls(*box);
    create_range_zoom_controls(*box);

    frame->add(*box);
    return frame;
}

Gtk::Widget*
PayloadSettingsTab::
create_focus_controls_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Focus");

    auto hbox = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 5);
    hbox->set_margin_top(10);
    hbox->set_margin_bottom(10);
    hbox->set_margin_start(10);
    hbox->set_margin_end(10);

    create_range_eo_speed_focus(*hbox);

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    box->pack_start(*Gtk::make_managed<Gtk::Label>("Continuous"), Gtk::PACK_SHRINK);
    add_button_to_box(*box, "Focus In", CAM_FOCUS_CONTINIOUS, 1);
    add_button_to_box(*box, "Stop", CAM_FOCUS_CONTINIOUS, 0);
    add_button_to_box(*box, "Focus Out", CAM_FOCUS_CONTINIOUS, -1);

    hbox->pack_start(*box, Gtk::PACK_EXPAND_WIDGET);

    add_button_to_box(*hbox, "Auto", CAM_FOCUS_AUTO, 1);
    
    frame->add(*hbox);
    return frame;
}

Gtk::Widget* 
PayloadSettingsTab::
create_exposure_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Exposure");

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    box->pack_start(*create_combo_box(ae_mode_combo, "Mode", AE_mode_list, sizeof(AE_mode_list)/sizeof(AE_mode_list[0]), CAM_AE_MODE), Gtk::PACK_SHRINK);

    auto hbox = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 10);

    hbox->set_margin_top(10);
    hbox->set_margin_bottom(10);
    hbox->set_margin_start(10);
    hbox->set_margin_end(10);

    hbox->pack_start(*create_combo_box(shutter_combo, "Shutter", shutter_mode_list, sizeof(shutter_mode_list)/sizeof(shutter_mode_list[0]), CAM_SHUTTER), Gtk::PACK_EXPAND_WIDGET);
    hbox->pack_start(*create_combo_box(iris_combo, "Iris", iris_mode_list, sizeof(iris_mode_list)/sizeof(iris_mode_list[0]), CAM_IRIS), Gtk::PACK_EXPAND_WIDGET);
    hbox->pack_start(*create_combo_box(gain_combo, "Gain", gain_mode_list, sizeof(gain_mode_list)/sizeof(gain_mode_list[0]), CAM_GAIN), Gtk::PACK_EXPAND_WIDGET);

    box->pack_start(*hbox, Gtk::PACK_SHRINK);

    frame->add(*box);
    return frame;
}

Gtk::Widget* 
PayloadSettingsTab::
create_white_balance_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("White Balance");

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    box->pack_start(*create_combo_box(wb_mode_combo, "Mode", white_balance_list, sizeof(white_balance_list)/sizeof(white_balance_list[0]), CAM_WHITE_BALANCE), Gtk::PACK_EXPAND_WIDGET);
    add_button_to_box(*box, "WB Trigger", CAM_WHITE_BALANCE_TRIGGER, 1.0);

    frame->add(*box);
    return frame;
}

Gtk::Widget* 
PayloadSettingsTab::
create_ir_palette_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("IR Camera");

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    box->pack_start(*create_combo_box(ir_palette_combo, "Palette", ir_palette_list, sizeof(ir_palette_list)/sizeof(ir_palette_list[0]), CAM_IR_PALETTE), Gtk::PACK_EXPAND_WIDGET);

    frame->add(*box);
    return frame;
}

Gtk::Widget* 
PayloadSettingsTab::
create_lrf_mode_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("LRF");

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    box->pack_start(*create_combo_box(lrf_mode_combo, "Frequency", lrf_mode_list, sizeof(lrf_mode_list)/sizeof(lrf_mode_list[0]), CAM_LRF_MODE), Gtk::PACK_EXPAND_WIDGET);

    frame->add(*box);
    return frame;
}

Gtk::Widget* 
PayloadSettingsTab::
create_osd_mode_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("OSD Mode");

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    box->pack_start(*create_combo_box(osd_mode_combo, "Mode", osd_mode_list, sizeof(osd_mode_list)/sizeof(osd_mode_list[0]), CAM_OSD_MODE), Gtk::PACK_EXPAND_WIDGET);

    frame->add(*box);
    return frame;
}

Gtk::Widget* 
PayloadSettingsTab::
create_image_flip_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Image Flip");

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    box->pack_start(*create_combo_box(image_flip_combo, "Mode", image_flip_list, sizeof(image_flip_list)/sizeof(image_flip_list[0]), CAM_IMAGE_FLIP), Gtk::PACK_EXPAND_WIDGET);

    frame->add(*box);
    return frame;
}

Gtk::Widget* 
PayloadSettingsTab::
create_info_row(const std::string& title, Gtk::Label*& label) {
    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    // box->set_margin_top(5);
    // box->set_margin_bottom(5);

    // Title label
    auto title_label = Gtk::make_managed<Gtk::Label>(title);
    title_label->set_size_request(150, -1);
    title_label->set_halign(Gtk::ALIGN_START);
    title_label->set_xalign(0.0);
    box->pack_start(*title_label, Gtk::PACK_SHRINK);

    // Value label
    label = Gtk::make_managed<Gtk::Label>("---");
    label->set_halign(Gtk::ALIGN_START);
    // label->set_markup("<span color='blue'>N/A</span>");
    label->set_markup("---");
    box->pack_start(*label, Gtk::PACK_EXPAND_WIDGET);

    return box;
}

void 
PayloadSettingsTab::
create_range_eo_speed_zoom(Gtk::Box& parent_box) {
    auto hbox = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);

    auto label = Gtk::make_managed<Gtk::Label>("EO Zoom Speed");

    eo_zoom_speed_range = Gtk::make_managed<Gtk::Scale>(Gtk::ORIENTATION_HORIZONTAL);
    eo_zoom_speed_range->set_range(0, 7);
    eo_zoom_speed_range->set_increments(1, 1);
    eo_zoom_speed_range->set_digits(0);
    eo_zoom_speed_range->signal_value_changed().connect(
        sigc::mem_fun(*this, &PayloadSettingsTab::on_eo_zoom_speed_changed)
    );

    hbox->pack_start(*label, Gtk::PACK_SHRINK);
    hbox->pack_start(*eo_zoom_speed_range, Gtk::PACK_EXPAND_WIDGET);

    parent_box.pack_start(*hbox, Gtk::PACK_SHRINK);
}

void 
PayloadSettingsTab::
create_continuous_zoom_controls(Gtk::Box& parent_box) {
    auto cont_box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    cont_box->pack_start(*Gtk::make_managed<Gtk::Label>("Continuous"), Gtk::PACK_SHRINK);

    add_button_to_box(*cont_box, "Zoom In", CAM_ZOOM_CONTINIOUS, 1.0);
    add_button_to_box(*cont_box, "Stop", CAM_ZOOM_CONTINIOUS, 0);
    add_button_to_box(*cont_box, "Zoom Out", CAM_ZOOM_CONTINIOUS, -1.0);
    parent_box.pack_start(*cont_box, Gtk::PACK_SHRINK);
}

void 
PayloadSettingsTab::
create_step_zoom_controls(Gtk::Box& parent_box) {
    auto step_box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    step_box->pack_start(*Gtk::make_managed<Gtk::Label>("Step               "), Gtk::PACK_SHRINK);
    add_button_to_box(*step_box, "Zoom In", CAM_ZOOM_STEP, 1);
    add_button_to_box(*step_box, "Zoom Out", CAM_ZOOM_STEP, -1);
    parent_box.pack_start(*step_box, Gtk::PACK_SHRINK);
}

void 
PayloadSettingsTab::
create_range_zoom_controls(Gtk::Box& parent_box) {
    auto hbox = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);

    auto label = Gtk::make_managed<Gtk::Label>("Range");

    zoom_range = Gtk::make_managed<Gtk::Scale>(Gtk::ORIENTATION_HORIZONTAL);
    zoom_range->set_range(0, 100);
    zoom_range->signal_value_changed().connect(
        sigc::mem_fun(*this, &PayloadSettingsTab::on_zoom_range_changed)
    );

    hbox->pack_start(*label, Gtk::PACK_SHRINK);
    hbox->pack_start(*zoom_range, Gtk::PACK_EXPAND_WIDGET);

    parent_box.pack_start(*hbox, Gtk::PACK_SHRINK);
}

void 
PayloadSettingsTab::
create_range_eo_speed_focus(Gtk::Box& parent_box) {
    auto hbox = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);

    auto label = Gtk::make_managed<Gtk::Label>("EO Focus Speed");

    eo_focus_speed_range = Gtk::make_managed<Gtk::Scale>(Gtk::ORIENTATION_HORIZONTAL);
    eo_focus_speed_range->set_range(0, 7);
    eo_focus_speed_range->set_increments(1, 1);
    eo_focus_speed_range->set_digits(0);
    eo_focus_speed_range->signal_value_changed().connect(
        sigc::mem_fun(*this, &PayloadSettingsTab::on_eo_focus_speed_changed)
    );

    hbox->pack_start(*label, Gtk::PACK_SHRINK);
    hbox->pack_start(*eo_focus_speed_range, Gtk::PACK_EXPAND_WIDGET);

    parent_box.pack_start(*hbox, Gtk::PACK_SHRINK);
}

// ===== Helper Methods =====
Gtk::Widget* 
PayloadSettingsTab::
create_gimbal_control_speed_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Control Speed");

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    speed_gimbal_range = Gtk::make_managed<Gtk::Scale>(Gtk::ORIENTATION_VERTICAL);
    speed_gimbal_range->set_range(1, 180);
    speed_gimbal_range->set_inverted(true);
    speed_gimbal_range->set_value(20);
    speed_gimbal = 20.0;
    speed_gimbal_range->signal_value_changed().connect(sigc::mem_fun(*this, &PayloadSettingsTab::on_speed_gimbal_changed));
    box->pack_start(*speed_gimbal_range, Gtk::PACK_SHRINK);

    auto grid = Gtk::make_managed<Gtk::Grid>();
    grid->set_row_spacing(10);
    grid->set_column_spacing(10);
    grid->set_margin_top(10);
    grid->set_margin_bottom(10);
    grid->set_margin_start(10);
    grid->set_margin_end(10);

    auto btn2 = Gtk::make_managed<Gtk::Button>("Up");
    grid->attach(*btn2, 1, 0, 1, 1);

    auto btn4 = Gtk::make_managed<Gtk::Button>("Left");
    auto btn5 = Gtk::make_managed<Gtk::Button>("Home");
    auto btn6 = Gtk::make_managed<Gtk::Button>("Right");
    grid->attach(*btn4, 0, 1, 1, 1);
    grid->attach(*btn5, 1, 1, 1, 1);
    grid->attach(*btn6, 2, 1, 1, 1);

    auto btn8 = Gtk::make_managed<Gtk::Button>("Down");
    grid->attach(*btn8, 1, 2, 1, 1);

    // Add signal
    btn5->signal_clicked().connect([this]() {
        double params[1] = {4.0};
        on_button_clicked(GIMBAL_MODE, params);
    });

    bind_control_gimbal_button(btn2, GIMBAL_CONTROL_TILT,  1.0);  // Up
    bind_control_gimbal_button(btn8, GIMBAL_CONTROL_TILT, -1.0);  // Down
    bind_control_gimbal_button(btn4, GIMBAL_CONTROL_PAN,  -1.0); // Left
    bind_control_gimbal_button(btn6, GIMBAL_CONTROL_PAN,   1.0); // Right
    
    box->pack_start(*grid, Gtk::PACK_SHRINK);

    frame->add(*box);
    return frame;
}

Gtk::Widget* 
PayloadSettingsTab::
create_gimbal_control_angle_group() {
    auto frame = Gtk::make_managed<Gtk::Frame>("Control Angle");

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_VERTICAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);

    auto hbox1 = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    auto label1 = Gtk::make_managed<Gtk::Label>("Pitch");
    pitch_angle_gimbal_range = Gtk::make_managed<Gtk::Scale>(Gtk::ORIENTATION_HORIZONTAL);
    pitch_angle_gimbal_range->set_range(-90, 90);
    pitch_angle_gimbal_range->set_value(0);
    pitch_angle_gimbal_range->signal_value_changed().connect(sigc::mem_fun(*this, &PayloadSettingsTab::on_angle_gimbal_changed));
    hbox1->pack_start(*label1, Gtk::PACK_SHRINK);
    hbox1->pack_start(*pitch_angle_gimbal_range, Gtk::PACK_EXPAND_WIDGET);

    auto hbox2 = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    auto label2 = Gtk::make_managed<Gtk::Label>("Roll  ");
    roll_angle_gimbal_range = Gtk::make_managed<Gtk::Scale>(Gtk::ORIENTATION_HORIZONTAL);
    roll_angle_gimbal_range->set_range(-45, 45);
    roll_angle_gimbal_range->set_value(0);
    roll_angle_gimbal_range->signal_value_changed().connect(sigc::mem_fun(*this, &PayloadSettingsTab::on_angle_gimbal_changed));
    hbox2->pack_start(*label2, Gtk::PACK_SHRINK);
    hbox2->pack_start(*roll_angle_gimbal_range, Gtk::PACK_EXPAND_WIDGET);

    auto hbox3 = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    auto label3 = Gtk::make_managed<Gtk::Label>("Yaw  ");
    yaw_angle_gimbal_range = Gtk::make_managed<Gtk::Scale>(Gtk::ORIENTATION_HORIZONTAL);
    yaw_angle_gimbal_range->set_range(-180, 180);
    yaw_angle_gimbal_range->set_value(0);
    yaw_angle_gimbal_range->signal_value_changed().connect(sigc::mem_fun(*this, &PayloadSettingsTab::on_angle_gimbal_changed));
    hbox3->pack_start(*label3, Gtk::PACK_SHRINK);
    hbox3->pack_start(*yaw_angle_gimbal_range, Gtk::PACK_EXPAND_WIDGET);

    box->pack_start(*hbox1, Gtk::PACK_SHRINK);
    box->pack_start(*hbox2, Gtk::PACK_SHRINK);
    box->pack_start(*hbox3, Gtk::PACK_SHRINK);
    
    frame->add(*box);
    return frame;
}

Gtk::Widget* 
PayloadSettingsTab::
create_combo_box(Gtk::ComboBoxText*& combo, const std::string& title, const list_struct_t* buttons, int size, _index_notify index) {

    auto box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 5);
    box->set_margin_top(10);
    box->set_margin_bottom(10);
    box->set_margin_start(10);
    box->set_margin_end(10);
    box->pack_start(*Gtk::make_managed<Gtk::Label>(title), Gtk::PACK_SHRINK);

    combo = Gtk::make_managed<Gtk::ComboBoxText>();
    for (int i = 0; i < size; i++) {
        combo->append(buttons[i].label);
    }
    combo->signal_changed().connect([this, combo, buttons, index]() {
        int active = combo->get_active_row_number();
        if (active >= 0) {
            double value = buttons[active].value;
            double params[1] = {value};
            on_button_clicked(index, params);
        }
    });
    combo->set_active(0);
    box->pack_start(*combo, Gtk::PACK_EXPAND_WIDGET);

    return box;
}

void 
PayloadSettingsTab::
update_combo_by_value(Gtk::ComboBoxText* combo, const list_struct_t* list, int size, double value){
    if(combo == nullptr) return;
    int active = combo->get_active_row_number();
    if (active >= 0 && list[active].value == value) return;

    for (int i = 0; i < size; i++) {
        if (list[i].value == value) {
            combo->set_active(i);
            return;
        }
    }
    combo->set_active(-1);
}

void 
PayloadSettingsTab::
add_button_to_box(Gtk::Box& box, const std::string& label, _index_notify index, double param) {
    auto button = Gtk::make_managed<Gtk::Button>(label);
    button->signal_clicked().connect([this, index, param]() {
        double params[1] = {param};
        on_button_clicked(index, params);
    });

    box.pack_start(*button, Gtk::PACK_EXPAND_WIDGET);
}

// ===== Event Handlers =====
void 
PayloadSettingsTab::
on_button_clicked(_index_notify index, double* value) {
    m_signal_button_clicked.emit(index, value);
}

void 
PayloadSettingsTab::
on_eo_zoom_speed_changed() {
    if (eo_zoom_speed_range) {
        double value = eo_zoom_speed_range->get_value();
        double params[1] = {value};
        on_button_clicked(CAM_EO_SPEED_ZOOM, params);
    }
}

void 
PayloadSettingsTab::
on_zoom_range_changed() {
    if (zoom_range) {
        double value = zoom_range->get_value();
        double params[1] = {value};
        on_button_clicked(CAM_ZOOM_RANGE, params);
    }
}

void 
PayloadSettingsTab::
on_eo_focus_speed_changed() {
    if (eo_focus_speed_range) {
        double value = eo_focus_speed_range->get_value();
        double params[1] = {value};
        on_button_clicked(CAM_EO_SPEED_FOCUS, params);
    }
}

void 
PayloadSettingsTab::
on_speed_gimbal_changed() {
    if (speed_gimbal_range) {
        speed_gimbal = speed_gimbal_range->get_value();
    }
}

void 
PayloadSettingsTab::
on_angle_gimbal_changed() {
    float _pitch = 0.0;
    float _roll  = 0.0;
    float _yaw   = 0.0;
    if (pitch_angle_gimbal_range) {
        _pitch = pitch_angle_gimbal_range->get_value();
    }
    if (roll_angle_gimbal_range) {
        _roll = roll_angle_gimbal_range->get_value();
    }
    if (yaw_angle_gimbal_range) {
        _yaw = yaw_angle_gimbal_range->get_value();
    }
    double params[3] = {_pitch, _roll, _yaw};
    on_button_clicked(GIMBAL_CONTROL_ANGLE, params);
}

void 
PayloadSettingsTab::
bind_control_gimbal_button(Gtk::Button* button, _index_notify index, double value) {
    button->signal_pressed().connect([this, button, index, value]() {
        // Create timer to send continuously every 100ms
        m_button_timers[button] = Glib::signal_timeout().connect(
            [this, index, value]() {
                double params[1] = {speed_gimbal * value};
                on_button_clicked(index, params);
                return true;
            },
            100 // ms
        );
    });

    // Released
    button->signal_released().connect([this, button, index]() {
        auto it = m_button_timers.find(button);
        if (it != m_button_timers.end() && it->second.connected()) {
            it->second.disconnect(); 
        }
        double params[1] = {0.0};
        on_button_clicked(index, params);
    });
}

// ===== Signal Accessors =====
sigc::signal<void, _index_notify, double*> PayloadSettingsTab::signal_button_clicked() {
    return m_signal_button_clicked;
}

void
PayloadSettingsTab::
update_storage_info(int status, double total, double used, double available){
    if (storage_info != nullptr) {
        if(status == 0){
            storage_info->set_markup("No SD Card");
        }
        else{
            char buf_text[256] = {0};
            sprintf(buf_text, "%.2f GB free of %.2f GB", available/1024.0, total/1024.0);
            storage_info->set_markup(buf_text);

        }
    }
}

void
PayloadSettingsTab::
update_capture_info(int img_status, int video_status, int img_count, int rec_time_ms){
    rec_status = video_status;
    if (capture_info != nullptr) {
        char buf_text[256] = {0};
        sprintf(buf_text, "%d", img_count);
        capture_info->set_markup(buf_text);
    }
    if (record_info != nullptr) {
        char buf_text[256] = {0};
        sprintf(buf_text, "%02d:%02d:%02d", (rec_time_ms/1000)/3600, ((rec_time_ms/1000)/3600)/60, (rec_time_ms/1000)%60);
        record_info->set_markup(buf_text);
    }
}

void
PayloadSettingsTab::
query_payload_param(){
    double params[1] = {0.0};
    on_button_clicked(QUERY_PAYLOAD_PARAM, params);
}

void
PayloadSettingsTab::
update_gimbal_attitude(float pitch, float roll, float yaw){
    float _p = pitch;
    float _r = roll;
    float _y = yaw;
    Glib::signal_idle().connect_once([this, _p, _r, _y](){
        char buf_text[256] = {0};
        sprintf(buf_text, "%.2f", _p);
        pitch_angle_info->set_markup(buf_text);

        sprintf(buf_text, "%.2f", _r);
        roll_angle_info->set_markup(buf_text);

        sprintf(buf_text, "%.2f", _y);
        yaw_angle_info->set_markup(buf_text);
    });
}

void
PayloadSettingsTab::
update_payload_status(double* params){
    int _index = params[0];
    double _value = params[1];

    Glib::signal_idle().connect_once([this, _index, _value]() {
        switch (_index){
            case PARAM_GIMBAL_MODE:{
                if(_value == 4) break;
                if (gimbal_mode_info != nullptr) {
                    if(_value == 0) gimbal_mode_info->set_markup("OFF");
                    else if(_value == 1) gimbal_mode_info->set_markup("LOCK");
                    else if(_value == 2) gimbal_mode_info->set_markup("FOLLOW");
                    else if(_value == 3) gimbal_mode_info->set_markup("MAPPING");
                    else gimbal_mode_info->set_markup("---");

                }
                
                update_combo_by_value(gimbal_mode_combo, gimbal_mode_list, sizeof(gimbal_mode_list)/sizeof(gimbal_mode_list[0]), _value);

                break;
            }
            case PARAM_CAM_VIEW_MODE:{
                if (view_mode_info != nullptr) {
                    if(_value == 0) view_mode_info->set_markup("EO/IR");
                    else if(_value == 1) view_mode_info->set_markup("EO");
                    else if(_value == 2) view_mode_info->set_markup("IR");
                    else if(_value == 3) view_mode_info->set_markup("IR/EO");
                    else if(_value == 4) view_mode_info->set_markup("SYNC");
                    else if(_value == 6) view_mode_info->set_markup("SIDE BY SIDE");
                    else view_mode_info->set_markup("---");
                }

                update_combo_by_value(view_mode_combo, cam_view_list, sizeof(cam_view_list)/sizeof(cam_view_list[0]), _value);
                break;
            }
            case PARAM_CAM_REC_SOURCE:{
                if (record_src_info != nullptr) {
                    if(_value == 0.0) record_src_info->set_markup("Both EO/IR");
                    else if(_value == 1.0) record_src_info->set_markup("EO");
                    else if(_value == 2.0) record_src_info->set_markup("IR");
                    else if(_value == 5.0) record_src_info->set_markup("OSD");
                    else record_src_info->set_markup("---");
                }
                update_combo_by_value(rec_src_combo, cam_src_list, sizeof(cam_src_list)/sizeof(cam_src_list[0]), _value);
                break;
            }
            case PARAM_EO_ZOOM_LEVEL:{
                if (eo_zoom_level_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    eo_zoom_level_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_IR_ZOOM_LEVEL:{
                if (ir_zoom_level_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    ir_zoom_level_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_CAM_IR_TYPE:{
                if (ir_type_info != nullptr) {
                    if(_value == 0) ir_type_info->set_markup("NO THERMAL CAMERA");
                    else if(_value == 1) ir_type_info->set_markup("G1");
                    else if(_value == 2) ir_type_info->set_markup("F1");
                    else ir_type_info->set_markup("---");
                }
                
                break;
            }
            case PARAM_CAM_IR_PALETTE_ID:{
                if (ir_palette_info != nullptr) {
                    if(_value == 0) ir_palette_info->set_markup("PALETTE 1");
                    else if(_value == 1) ir_palette_info->set_markup("PALETTE 2");
                    else if(_value == 2) ir_palette_info->set_markup("PALETTE 3");
                    else if(_value == 3) ir_palette_info->set_markup("PALETTE 4");
                    else if(_value == 4) ir_palette_info->set_markup("PALETTE 5");
                    else if(_value == 5) ir_palette_info->set_markup("PALETTE 6");
                    else if(_value == 6) ir_palette_info->set_markup("PALETTE 7");
                    else if(_value == 7) ir_palette_info->set_markup("PALETTE 8");
                    else if(_value == 8) ir_palette_info->set_markup("PALETTE 9");
                    else if(_value == 9) ir_palette_info->set_markup("PALETTE 10");
                    else ir_palette_info->set_markup("---");
                    update_combo_by_value(ir_palette_combo, ir_palette_list, sizeof(ir_palette_list)/sizeof(ir_palette_list[0]), _value);
                }
                break;
            }
            case PARAM_CAM_IR_FFC_MODE:{
                if (ir_ffc_mode_info != nullptr) {
                    if(_value == 0) ir_ffc_mode_info->set_markup("Manual");
                    else if(_value == 1) ir_ffc_mode_info->set_markup("Auto");
                    else ir_ffc_mode_info->set_markup("---");
                }
                break;
            }
            case PARAM_IR_TEMP_MAX:{
                if (ir_temp_max_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f°C", _value);
                    ir_temp_max_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_IR_TEMP_MIN:{
                if (ir_temp_min_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f°C", _value);
                    ir_temp_min_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_IR_TEMP_MEAN:{
                if (ir_temp_mean_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f°C", _value);
                    ir_temp_mean_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_LRF_RANGE:{
                if (lrf_range_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2fm", _value);
                    lrf_range_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_LRF_OFSET_X:{
                if (lrf_offset_x_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    lrf_offset_x_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_LRF_OFSET_Y:{
                if (lrf_offset_y_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    lrf_offset_y_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_TARGET_COOR_LON:{
                if (target_gps_lon_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    target_gps_lon_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_TARGET_COOR_LAT:{
                if (target_gps_lat_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    target_gps_lat_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_TARGET_COOR_ALT:{
                if (target_gps_alt_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    target_gps_alt_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_PAYLOAD_GPS_LON:{
                if (payload_gps_lon_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    payload_gps_lon_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_PAYLOAD_GPS_LAT:{
                if (payload_gps_lat_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    payload_gps_lat_info->set_markup(buf_text);
                }
                break;
            }
            case PARAM_PAYLOAD_GPS_ALT:{
                if (payload_gps_alt_info != nullptr) {
                    char buf_text[256] = {0};
                    sprintf(buf_text, "%.2f", _value);
                    payload_gps_alt_info->set_markup(buf_text);
                }
                break;
            }
            default: break;
        }
    });
}


void
PayloadSettingsTab::
update_payload_param(char* index, double value){
    std::string _index = index;
    double _value = value;

    Glib::signal_idle().connect_once([this, _index, _value]() {
        if(_index == PAYLOAD_CAMERA_VIEW_SRC){
            // do nothing
        }
        else if(_index == PAYLOAD_CAMERA_RECORD_SRC){
            // do nothing
        }
        else if(_index == PAYLOAD_CAMERA_EO_ZOOM_SPEED){
            if(eo_zoom_speed_range != nullptr)
                eo_zoom_speed_range->set_value((int)_value);
        }
        else if(_index == PAYLOAD_CAMERA_EO_FOCUS_SPEED){
            if(eo_focus_speed_range != nullptr)
                eo_focus_speed_range->set_value((int)_value);
        }
        else if(_index == PAYLOAD_CAMERA_VIDEO_AUTO_EXPOSURE){
            update_combo_by_value(ae_mode_combo, AE_mode_list, sizeof(AE_mode_list)/sizeof(AE_mode_list[0]), _value);
        }
        else if(_index == PAYLOAD_CAMERA_VIDEO_SHUTTER_SPEED){
            update_combo_by_value(shutter_combo, shutter_mode_list, sizeof(shutter_mode_list)/sizeof(shutter_mode_list[0]), _value);
        }
        else if(_index == PAYLOAD_CAMERA_VIDEO_APERTURE_VALUE){
            update_combo_by_value(iris_combo, iris_mode_list, sizeof(iris_mode_list)/sizeof(iris_mode_list[0]), _value);
        }
        else if(_index == PAYLOAD_CAMERA_EO_GAIN_LS){
            update_combo_by_value(gain_combo, gain_mode_list, sizeof(gain_mode_list)/sizeof(gain_mode_list[0]), _value);
        }
        else if(_index == PAYLOAD_CAMERA_VIDEO_WHITE_BALANCE){
            update_combo_by_value(wb_mode_combo, white_balance_list, sizeof(white_balance_list)/sizeof(white_balance_list[0]), _value);
        }
        else if(_index == PAYLOAD_CAMERA_IR_PALETTE){
            update_combo_by_value(ir_palette_combo, ir_palette_list, sizeof(ir_palette_list)/sizeof(ir_palette_list[0]), _value);
        }
        else if(_index == PAYLOAD_LRF_MODE){
            update_combo_by_value(lrf_mode_combo, lrf_mode_list, sizeof(lrf_mode_list)/sizeof(lrf_mode_list[0]), _value);
        }
        else if(_index == PAYLOAD_CAMERA_VIDEO_OSD_MODE){
            update_combo_by_value(osd_mode_combo, osd_mode_list, sizeof(osd_mode_list)/sizeof(osd_mode_list[0]), _value);
        }
        else if(_index == PAYLOAD_CAMERA_VIDEO_FLIP){
            update_combo_by_value(image_flip_combo, image_flip_list, sizeof(image_flip_list)/sizeof(image_flip_list[0]), _value);
        }
        else if(_index == PAYLOAD_CAMERA_GIMBAL_MODE){
            // do nothing
        }
    });
}