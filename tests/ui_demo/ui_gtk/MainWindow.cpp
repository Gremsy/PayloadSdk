#include <MainWindow.h>



MainWindow::MainWindow(int width, int height) {
    set_title("Payload UI Demo");
    set_default_size(width, height);

    // Config main box
    main_box.set_orientation(Gtk::ORIENTATION_HORIZONTAL);
    add(main_box);

    #if 0
    side_box.set_orientation(Gtk::ORIENTATION_VERTICAL);
    side_box.set_margin_top(10);
    side_box.set_margin_bottom(10);
    side_box.set_margin_start(10);
    side_box.set_margin_end(10);
    side_box.set_spacing(10);
    side_box.set_valign(Gtk::ALIGN_START);
    
    // Create buttons with event handlers
    auto control_btn = Gtk::make_managed<Gtk::Button>("Control");
    auto video_btn = Gtk::make_managed<Gtk::Button>("Video");
    auto info_btn = Gtk::make_managed<Gtk::Button>("Information");
    
    control_btn->signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_control_button_clicked));
    video_btn->signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_video_button_clicked));
    info_btn->signal_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_information_button_clicked));
    
    side_box.pack_start(*control_btn, Gtk::PACK_SHRINK);
    side_box.pack_start(*video_btn, Gtk::PACK_SHRINK);
    side_box.pack_start(*info_btn, Gtk::PACK_SHRINK);

    // Add side_box to main_box
    main_box.pack_start(side_box, Gtk::PACK_SHRINK);
    #endif

    // Cấu hình phần nội dung tab
    tab_content.set_orientation(Gtk::ORIENTATION_VERTICAL);
    tab_content.set_margin_top(10);
    tab_content.set_margin_bottom(10);
    tab_content.set_margin_start(10);
    tab_content.set_margin_end(10);

    // Tạo payload tab
    payload_tab = Gtk::make_managed<PayloadSettingsTab>();
    payload_tab->signal_button_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_payload_button_clicked));

    tab_content.pack_start(*payload_tab, Gtk::PACK_EXPAND_WIDGET);

    // Thêm tab_content vào main_box
    main_box.pack_start(tab_content);

    show_all_children();
    // payload_tab->query_payload_param();
}

void 
MainWindow::
regUICommandChanged(ui_callback_t func) {
    __notifyUICommandChanged = func;
}

void 
MainWindow::
on_payload_button_clicked(_index_notify index, double* param) {
    // double params[1] = {param};
    if(__notifyUICommandChanged)
        __notifyUICommandChanged(index, param);
}

void
MainWindow::
update_storage_info(int status, double total, double used, double available){
    if (payload_tab) {
        payload_tab->update_storage_info(status, total, used, available);
    }
}

void
MainWindow::
update_capture_info(int img_status, int video_status, int img_count, int rec_time_ms){
    if (payload_tab) {
        payload_tab->update_capture_info(img_status, video_status, img_count, rec_time_ms);
    }
}

void
MainWindow::
update_gimbal_attitude(float pitch, float roll, float yaw){
    if (payload_tab) {
        payload_tab->update_gimbal_attitude(pitch, roll, yaw);
    }
}

void
MainWindow::
update_payload_status(double* params){
    if (payload_tab) {
        payload_tab->update_payload_status(params);
    }
}

void
MainWindow::
update_payload_param(char* index, double value){
    if (payload_tab) {
        payload_tab->update_payload_param(index, value);
    }
}