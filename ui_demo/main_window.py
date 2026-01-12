#!/usr/bin/env python3
"""
Main Window for Payload SDK UI Demo
Based on C++ PayloadSdk UI implementation
"""

import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk, Gdk, GLib

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'libs'))

from payload_settings_tab import PayloadSettingsTab
from config import ConnectionConfig

# Connection callback types
UI_COMMAND_CALLBACK = None
UI_CONNECT_CALLBACK = None


class MainWindow(Gtk.Window):
    """Main application window"""

    def __init__(self, width=1920, height=1080, is_mb1=False):
        super().__init__(title="Payload UI Demo - Python")
        self.set_default_size(width, height)

        self.is_connected = False
        self.is_mb1 = is_mb1
        self._notify_ui_command_changed = None
        self._notify_ui_connect_command_changed = None

        # Main vertical box
        self.main_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.add(self.main_box)

        # IP connection frame
        self.main_box.pack_start(self._create_connect_ip(), False, False, 0)

        # Scrolled window for tab content
        scrolled_window = Gtk.ScrolledWindow()
        scrolled_window.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        scrolled_window.set_vexpand(True)
        scrolled_window.set_hexpand(True)

        # Tab content
        self.tab_content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.tab_content.set_margin_top(5)
        self.tab_content.set_margin_bottom(10)
        self.tab_content.set_hexpand(True)
        self.tab_content.set_vexpand(True)

        # Payload settings tab (pass is_mb1 flag)
        self.payload_tab = PayloadSettingsTab(is_mb1=is_mb1)
        self.payload_tab.connect_button_clicked(self._on_payload_button_clicked)
        self.payload_tab.set_sensitive(False)
        self.payload_tab.set_hexpand(True)

        self.tab_content.pack_start(self.payload_tab, True, True, 0)

        scrolled_window.add(self.tab_content)
        self.main_box.pack_start(scrolled_window, True, True, 0)

        self.show_all()

    def _create_connect_ip(self):
        """Create IP connection frame"""
        frame = Gtk.Frame(label="Connection")
        frame.set_halign(Gtk.Align.FILL)
        frame.set_valign(Gtk.Align.CENTER)
        frame.set_margin_top(10)
        frame.set_margin_start(10)
        frame.set_margin_end(10)
        frame.set_margin_bottom(5)

        ip_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        ip_box.set_margin_top(10)
        ip_box.set_margin_bottom(10)
        ip_box.set_margin_start(10)
        ip_box.set_margin_end(10)

        # IP Entry
        self.ip_entry = Gtk.Entry()
        self.ip_entry.set_size_request(300, -1)
        self.ip_entry.set_placeholder_text("Enter your payload IP address")
        self.ip_entry.set_text(ConnectionConfig.UDP_IP_TARGET)  # Default IP from config
        self.ip_entry.connect("changed", self._on_ip_entry_changed)

        ip_box.pack_start(self.ip_entry, True, True, 0)

        # Connect button
        self.btn_connect = Gtk.Button(label="Connect")
        self.btn_connect.set_size_request(150, -1)
        self.btn_connect.connect("clicked", self._on_connect_clicked)
        ip_box.pack_start(self.btn_connect, False, False, 0)

        # Connection status label
        self.connect_info = Gtk.Label()
        self.connect_info.set_markup("<span color='red'>  Disconnected </span>")
        ip_box.pack_start(self.connect_info, False, False, 0)

        frame.add(ip_box)
        return frame

    def _on_connect_clicked(self, button):
        """Handle connect button click"""
        if not self.is_connected:
            ip = self.ip_entry.get_text()
            self._on_connect_button_clicked("CONNECT_PAYLOAD", ip)
        else:
            self._on_connect_button_clicked("DISCONNECT_PAYLOAD", "")

    def _on_ip_entry_changed(self, entry):
        """Handle IP entry change"""
        if self.payload_tab and self.ip_entry:
            ip = self.ip_entry.get_text()
            if ip:
                self.payload_tab.update_rtsp_url_from_ip(ip)

    def reg_ui_command_changed(self, func):
        """Register UI command callback"""
        self._notify_ui_command_changed = func

    def reg_ui_connect_command_changed(self, func):
        """Register UI connect command callback"""
        self._notify_ui_connect_command_changed = func

    def _on_payload_button_clicked(self, index, params):
        """Handle payload button click"""
        if self._notify_ui_command_changed:
            self._notify_ui_command_changed(index, params)

    def _on_connect_button_clicked(self, index, param):
        """Handle connect button click"""
        if self._notify_ui_connect_command_changed:
            self._notify_ui_connect_command_changed(index, param)

    def send_connected(self):
        """Update UI for connected state"""
        self.is_connected = True
        if self.connect_info:
            self.connect_info.set_markup("<span color='green'>  Connected </span>")
        if self.btn_connect:
            self.btn_connect.set_label("Disconnect")
        if self.payload_tab:
            self.payload_tab.set_sensitive(True)
            self.payload_tab.send_connected()

    def send_disconnected(self):
        """Update UI for disconnected state"""
        self.is_connected = False
        if self.connect_info:
            self.connect_info.set_markup("<span color='red'>  Disconnected </span>")
        if self.btn_connect:
            self.btn_connect.set_label("Connect")
        if self.payload_tab:
            self.payload_tab.set_sensitive(False)
            self.payload_tab.send_disconnected()

    def update_storage_info(self, status, total, used, available):
        """Update storage info"""
        if self.payload_tab:
            self.payload_tab.update_storage_info(status, total, used, available)

    def update_capture_info(self, img_status, video_status, img_count, rec_time_ms):
        """Update capture info"""
        if self.payload_tab:
            self.payload_tab.update_capture_info(img_status, video_status, img_count, rec_time_ms)

    def update_gimbal_attitude(self, pitch, roll, yaw):
        """Update gimbal attitude"""
        if self.payload_tab:
            self.payload_tab.update_gimbal_attitude(pitch, roll, yaw)

    def update_payload_status(self, params):
        """Update payload status"""
        if self.payload_tab:
            self.payload_tab.update_payload_status(params)

    def update_payload_param(self, index, value):
        """Update payload parameter"""
        if self.payload_tab:
            self.payload_tab.update_payload_param(index, value)

    def update_url_streaming(self, url):
        """Update streaming URL"""
        if self.payload_tab:
            self.payload_tab.update_url_streaming(url)

    def update_gimbal_mode_from_string(self, mode_string):
        """Update gimbal mode from mode string (e.g., 'LOCK_MODE', 'FOLLOW_MODE')"""
        if self.payload_tab:
            self.payload_tab.update_gimbal_mode_from_string(mode_string)
