#include <MainWindow.h>



MainWindow::MainWindow(int width, int height) {
    set_title("Payload UI Demo");
    set_default_size(width, height);

    // Config main box
    main_box.set_orientation(Gtk::ORIENTATION_VERTICAL);
    add(main_box);

    main_box.pack_start(*create_connect_ip(), Gtk::PACK_SHRINK);

    // Create scrolled window for tab content
    auto scrolled_window = Gtk::manage(new Gtk::ScrolledWindow());
    scrolled_window->set_policy(Gtk::POLICY_AUTOMATIC, Gtk::POLICY_AUTOMATIC);
    scrolled_window->set_vexpand(true);
    scrolled_window->set_hexpand(true);

    tab_content.set_orientation(Gtk::ORIENTATION_VERTICAL);
    tab_content.set_margin_top(5);
    tab_content.set_margin_bottom(10);
    tab_content.set_hexpand(true);
    tab_content.set_vexpand(true);

    payload_tab = Gtk::manage(new PayloadSettingsTab());
    payload_tab->signal_button_clicked().connect(sigc::mem_fun(*this, &MainWindow::on_payload_button_clicked));
    payload_tab->set_sensitive(false);
    payload_tab->set_hexpand(true);

    tab_content.pack_start(*payload_tab, Gtk::PACK_EXPAND_WIDGET);

    scrolled_window->add(tab_content);
    main_box.pack_start(*scrolled_window, Gtk::PACK_EXPAND_WIDGET);

    show_all_children();
}

Gtk::Widget*
MainWindow::
create_connect_ip(){
    auto frame = Gtk::manage(new Gtk::Frame("IP"));
    frame->set_halign(Gtk::ALIGN_FILL);
    frame->set_valign(Gtk::ALIGN_CENTER);
    frame->set_margin_top(10);
    frame->set_margin_start(10);
    frame->set_margin_end(10);
    frame->set_margin_bottom(5);

    auto ip_box = Gtk::manage(new Gtk::Box(Gtk::ORIENTATION_HORIZONTAL, 10));
    ip_box->set_margin_top(10);
    ip_box->set_margin_bottom(10);
    ip_box->set_margin_start(10);
    ip_box->set_margin_end(10);

    ip_entry = Gtk::manage(new Gtk::Entry());
    ip_entry->set_size_request(300, -1);
    ip_entry->set_placeholder_text("Enter your payload IP address");
    ip_entry->set_text(udp_ip_target);
    ip_entry->signal_changed().connect(sigc::mem_fun(*this, &MainWindow::on_ip_entry_changed));

    ip_box->pack_start(*ip_entry, Gtk::PACK_EXPAND_WIDGET);

    btn_connect = Gtk::manage(new Gtk::Button("Connect"));
    btn_connect->set_size_request(150, -1);
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
    ip_box->pack_start(*btn_connect, Gtk::PACK_SHRINK);

    connect_info = Gtk::manage(new Gtk::Label("  Disconnected "));
    connect_info->set_markup("<span color='red'>  Disconnected </span>");
    ip_box->pack_start(*connect_info, Gtk::PACK_SHRINK);

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

void
MainWindow::
on_ip_entry_changed(){
    if (payload_tab && ip_entry) {
        std::string ip = ip_entry->get_text();
        // Only update if IP is not empty
        if (!ip.empty()) {
            payload_tab->update_rtsp_url_from_ip(ip);
        }
    }
}