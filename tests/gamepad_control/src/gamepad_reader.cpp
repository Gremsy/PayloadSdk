#include "gamepad_reader.h"

#include <linux/input.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <vector>

namespace fs = std::filesystem;

namespace {
const char* kButtonNames[] = {
    "A", "B", "X", "Y", "LB", "RB", "BACK", "START", "LOGITECH", "L3", "R3"};
const char* kAxisNames[] = {
    "LX", "LY", "LT", "RX", "RY", "RT", "DPAD_X", "DPAD_Y"};

constexpr int kPollTimeoutMs = 10;     // chu ky kiem tra khi dang doc
constexpr int kRescanIntervalMs = 1000;  // chu ky tim gamepad khi chua cam
constexpr int kStopCheckMs = 50;       // chia nho thoi gian cho de stop() nhanh
}  // namespace

GamepadReader::~GamepadReader() {
    stop();
}

void GamepadReader::setCallback(GamepadCallback cb) {
    std::lock_guard<std::mutex> lock(cbMutex_);
    callback_ = std::move(cb);
}

bool GamepadReader::start() {
    if (running_) return false;
    running_ = true;
    thread_ = std::thread(&GamepadReader::threadLoop, this);
    return true;
}

void GamepadReader::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

const char* GamepadReader::buttonName(int n) {
    return (n >= 0 && n < (int)(sizeof(kButtonNames) / sizeof(*kButtonNames))) ? kButtonNames[n] : "?";
}

const char* GamepadReader::axisName(int n) {
    return (n >= 0 && n < (int)(sizeof(kAxisNames) / sizeof(*kAxisNames))) ? kAxisNames[n] : "?";
}

bool GamepadReader::testBit(const unsigned long* bits, int bit) {
    const int bitsPerLong = 8 * sizeof(unsigned long);
    return (bits[bit / bitsPerLong] >> (bit % bitsPerLong)) & 1UL;
}

void GamepadReader::notify(const GamepadEvent& ev) {
    GamepadCallback cb;
    {
        std::lock_guard<std::mutex> lock(cbMutex_);
        cb = callback_;
    }
    if (cb) cb(ev);
}

// Tim /dev/input/eventN cung thiet bi voi jsN, roi kiem tra co BTN_GAMEPAD khong.
// Tra ve: 1 = la gamepad, 0 = khong phai, -1 = khong kiem tra duoc (thieu quyen...)
int GamepadReader::hasGamepadButtons(int jsIndex) {
    const fs::path sysDir = "/sys/class/input/js" + std::to_string(jsIndex) + "/device";
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator(sysDir, ec)) {
        const std::string name = entry.path().filename().string();
        if (name.rfind("event", 0) != 0) continue;

        int fd = open(("/dev/input/" + name).c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) return -1;

        const int nLongs = KEY_MAX / (8 * sizeof(unsigned long)) + 1;
        std::vector<unsigned long> keyBits(nLongs, 0);
        int ok = ioctl(fd, EVIOCGBIT(EV_KEY, nLongs * sizeof(unsigned long)), keyBits.data());
        close(fd);
        if (ok < 0) return -1;
        return testBit(keyBits.data(), BTN_GAMEPAD) ? 1 : 0;
    }
    return -1;
}

bool GamepadReader::nameLooksLikeGamepad(const std::string& n) {
    for (const char* key : {"Gamepad", "gamepad", "Logitech", "Xbox", "X-Box", "Joystick"})
        if (n.find(key) != std::string::npos) return true;
    return false;
}

// Quet tat ca jsN, tra ve duong dan gamepad dau tien tim thay (rong neu khong co)
std::string GamepadReader::findGamepad(std::string& outName) {
    for (int i = 0; i < 32; ++i) {
        const std::string path = "/dev/input/js" + std::to_string(i);
        int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;
        char name[128] = "";
        ioctl(fd, JSIOCGNAME(sizeof(name)), name);
        close(fd);

        int caps = hasGamepadButtons(i);
        if (caps == 1 || (caps == -1 && nameLooksLikeGamepad(name))) {
            outName = name;
            return path;
        }
    }
    return "";
}

// Vong lap chinh cua thread: tim gamepad -> doc -> neu rut ra thi tim lai
void GamepadReader::threadLoop() {
    while (running_) {
        std::string name;
        const std::string dev = findGamepad(name);
        int fd = dev.empty() ? -1 : open(dev.c_str(), O_RDONLY | O_NONBLOCK);

        if (fd < 0) {
            // Chua co gamepad: cho roi tim lai, van phan ung nhanh voi stop()
            for (int t = 0; t < kRescanIntervalMs && running_; t += kStopCheckMs)
                std::this_thread::sleep_for(std::chrono::milliseconds(kStopCheckMs));
            continue;
        }

        connected_ = true;
        GamepadEvent ev;
        ev.type = GamepadEventType::Connected;
        ev.deviceName = name;
        ev.devicePath = dev;
        notify(ev);

        runGamepad(fd);
        close(fd);

        connected_ = false;
        GamepadEvent dev_ev;
        dev_ev.type = GamepadEventType::Disconnected;
        notify(dev_ev);
    }
}

// Doc su kien tu gamepad cho den khi bi rut ra hoac stop()
void GamepadReader::runGamepad(int fd) {
    unsigned char numAxes = 0, numButtons = 0;
    ioctl(fd, JSIOCGAXES, &numAxes);
    ioctl(fd, JSIOCGBUTTONS, &numButtons);

    pollfd pfd{fd, POLLIN, 0};
    while (running_) {
        int ret = poll(&pfd, 1, kPollTimeoutMs);
        if (ret < 0) {
            if (errno == EINTR) continue;
            return;
        }
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return;  // bi rut ra
        if (ret == 0) continue;  // timeout, kiem tra lai running_

        js_event e;
        ssize_t n;
        while ((n = read(fd, &e, sizeof(e))) == sizeof(e)) {
            // Su kien JS_EVENT_INIT (trang thai ban dau) van duoc gui,
            // de ben ngoai biet trang thai hien tai ngay khi ket noi.
            const unsigned char type = e.type & ~JS_EVENT_INIT;

            GamepadEvent ev;
            if (type == JS_EVENT_BUTTON && e.number < numButtons) {
                ev.type = GamepadEventType::Button;
            } else if (type == JS_EVENT_AXIS && e.number < numAxes) {
                ev.type = GamepadEventType::Axis;
            } else {
                continue;
            }
            ev.number = e.number;
            ev.value = e.value;
            ev.timeMs = e.time;
            notify(ev);
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return;  // ENODEV khi rut
    }
}