#include <MainWindow.h>



MainWindow::MainWindow(int width, int height) {
    set_title("Payload UI Demo");
    set_default_size(width, height);

    // Config main box
    main_box.set_orientation(Gtk::ORIENTATION_VERTICAL);
    add(main_box);

    main_box.pack_start(*create_connect_ip(), Gtk::PACK_SHRINK);

    tab_content.set_orientation(Gtk::ORIENTATION_VERTICAL);
    tab_content.set_margin_top(10);
    tab_content.set_margin_bottom(10);
    tab_content.set_margin_start(10);
    tab_content.set_margin_end(10);

    payload_tab = Gtk::make_managed<PayloadSettingsTab>();
    payload_tab->signal_button_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_payload_button_clicked));
    payload_tab->set_sensitive(false);

    tab_content.pack_start(*payload_tab, Gtk::PACK_EXPAND_WIDGET);
    tab_content.set_hexpand(true);
    main_box.pack_start(tab_content);

    show_all_children();
}

Gtk::Widget* 
MainWindow::
create_connect_ip(){
    auto frame = Gtk::make_managed<Gtk::Frame>("IP");
    frame->set_halign(Gtk::ALIGN_START);
    frame->set_valign(Gtk::ALIGN_CENTER);
    frame->set_margin_top(10);
    frame->set_margin_start(30);
    frame->set_margin_end(10);

    auto ip_box = Gtk::make_managed<Gtk::Box>(Gtk::ORIENTATION_HORIZONTAL, 10);
    ip_box->set_margin_top(10);
    ip_box->set_margin_bottom(10);
    ip_box->set_margin_start(10);
    ip_box->set_margin_end(10);

    ip_entry = Gtk::make_managed<Gtk::Entry>();
    ip_entry->set_size_request(300, -1);
    ip_entry->set_placeholder_text("rtsp://example.com:554/stream");
    ip_entry->set_text("192.168.15.250");

    ip_box->pack_start(*ip_entry, Gtk::PACK_EXPAND_WIDGET);
    
    btn_connect = Gtk::make_managed<Gtk::Button>("Connect");
    ip_entry->set_size_request(150, -1);
    btn_connect->signal_clicked().connect([this]() {
        if(!is_connected){
            printf("debug 1\n");
            std::string ip = ip_entry->get_text();
            on_connect_button_clicked(CONNECT_PAYLOAD, ip.c_str());
        }
        else{
            printf("debug 2\n");
            on_connect_button_clicked(DISCONNECT_PAYLOAD, "");
        }

    });
    ip_box->pack_start(*btn_connect, Gtk::PACK_EXPAND_WIDGET);

    connect_info = Gtk::make_managed<Gtk::Label>("  Disconnected ");
    connect_info->set_markup("<span color='red'>  Disconnected </span>");
    ip_box->pack_start(*connect_info, Gtk::PACK_EXPAND_WIDGET);

    frame->add(*ip_box);
    return frame;
}

void 
MainWindow::
regUICommandChanged(ui_callback_t func) {
    __notifyUICommandChanged = func;
}

void 
MainWindow::
regUIConnectCommandChanged(ui_connect_callback_t func) {
    __notifyUIConnectCommandChanged = func;
}


void 
MainWindow::
on_payload_button_clicked(_index_notify index, double* param) {
    if(__notifyUICommandChanged)
        __notifyUICommandChanged(index, param);
}

void 
MainWindow::
on_connect_button_clicked(_index_notify index, const char* param) {
    if(__notifyUIConnectCommandChanged)
        __notifyUIConnectCommandChanged(index, param);
}

void
MainWindow::
send_connected(){
    is_connected = true;
    if(connect_info != nullptr){
        connect_info->set_markup("<span color='green'>  Connected </span>");
    }
    if(btn_connect != nullptr){
        btn_connect->set_label("Disconnect");
    }
    if(payload_tab != nullptr){
        payload_tab->set_sensitive(true);
        payload_tab->send_connected();
    }
}

void
MainWindow::
send_disconnected(){
    is_connected = false;
    if(connect_info != nullptr){
        connect_info->set_markup("<span color='red'>  Disconnected </span>");
    }
    if(btn_connect != nullptr){
        btn_connect->set_label("Connect");
    }
    if(payload_tab != nullptr){
        payload_tab->set_sensitive(false);
        payload_tab->send_disconnected();
    }
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

void
MainWindow::
update_url_streaming(char* url){
    if (payload_tab) {
        payload_tab->update_url_streaming(url);
    }
}