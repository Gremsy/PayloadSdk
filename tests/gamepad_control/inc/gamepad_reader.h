#ifndef GAMEPAD_READER_H_
#define GAMEPAD_READER_H_

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

// Nut cua F310 o che do X (driver xpad)
enum class GamepadButton : int {
    A = 0, B, X, Y, LB, RB, BACK, START, LOGITECH, L3, R3
};

// Truc cua F310 o che do X (driver xpad), gia tri -32767..32767
enum class GamepadAxis : int {
    LX = 0, LY, LT, RX, RY, RT, DPAD_X, DPAD_Y
};

enum class GamepadEventType {
    Connected,     // da ket noi gamepad (deviceName, devicePath)
    Disconnected,  // gamepad bi rut ra
    Button,        // number = so nut, value = 1 nhan / 0 nha
    Axis           // number = so truc, value = -32767..32767
};

struct GamepadEvent {
    GamepadEventType type = GamepadEventType::Button;
    int number = 0;
    int value = 0;
    uint32_t timeMs = 0;      // thoi gian cua driver (ms)
    std::string deviceName;   // chi co khi Connected
    std::string devicePath;   // chi co khi Connected
};

using GamepadCallback = std::function<void(const GamepadEvent&)>;

class GamepadReader {
public:
    GamepadReader() = default;
    ~GamepadReader();

    GamepadReader(const GamepadReader&) = delete;
    GamepadReader& operator=(const GamepadReader&) = delete;

    // Dang ky callback. Callback duoc goi tu thread rieng cua GamepadReader,
    // nen can giu ngan gon va bao ve du lieu dung chung (mutex/atomic).
    void setCallback(GamepadCallback cb);

    // Bat dau thread: tu dong tim gamepad, doc su kien, tu ket noi lai khi rut/cam.
    bool start();

    // Dung thread va cho thread ket thuc.
    void stop();

    bool isRunning() const { return running_; }
    bool isConnected() const { return connected_; }

    static const char* buttonName(int n);
    static const char* axisName(int n);

private:
    void threadLoop();
    void runGamepad(int fd);
    std::string findGamepad(std::string& outName);
    int hasGamepadButtons(int jsIndex);
    static bool nameLooksLikeGamepad(const std::string& n);
    static bool testBit(const unsigned long* bits, int bit);

    void notify(const GamepadEvent& ev);

    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    std::mutex cbMutex_;
    GamepadCallback callback_;
};

#endif