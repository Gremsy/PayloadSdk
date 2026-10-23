#ifndef GAMEPAD_CONTROLLER_H_
#define GAMEPAD_CONTROLLER_H_

#include <atomic>
#include <functional>
#include <string>

#include "gamepad_reader.h"


enum class Action {
    NONE = 0,

    // ---- Analog ----
    GIMBAL_PITCH,
    GIMBAL_YAW,
    GIMBAL_ROLL,
    ZOOM,

    // ---- One-shot ----
    GIMBAL_HOME,
    GIMBAL_MODE,
    IMAGE_CAPTURE,
    VIDEO_RECORD,
    CHANGE_VIEW,
    CHANGE_RECORD,
    TRACKING,

    ZOOM_TELE,
    ZOOM_WIDE,

    COUNT
};


using ActionCallback = std::function<void(Action)>;

struct GamepadInput {
    bool isButton;   // true = button, false = axis
    int  index;      // index in GamepadButton / GamepadAxis
};

inline constexpr int GAMEPAD_NUM_BUTTONS = (int)GamepadButton::R3 + 1;
inline constexpr int GAMEPAD_NUM_AXES    = (int)GamepadAxis::DPAD_Y + 1;

inline constexpr const char* DEFAULT_MAPPING_FILE = "gamepad_mapping.txt";

inline constexpr Action DEFAULT_BUTTON_MAP[GAMEPAD_NUM_BUTTONS] = {
    /* A        */ Action::GIMBAL_HOME,
    /* B        */ Action::IMAGE_CAPTURE,
    /* X        */ Action::VIDEO_RECORD,
    /* Y        */ Action::CHANGE_VIEW,
    /* LB       */ Action::CHANGE_RECORD,
    /* RB       */ Action::GIMBAL_MODE,
    /* BACK     */ Action::NONE,
    /* START    */ Action::NONE,
    /* LOGITECH */ Action::TRACKING,
    /* L3       */ Action::NONE,
    /* R3       */ Action::NONE,
};

inline constexpr Action DEFAULT_AXIS_MAP[GAMEPAD_NUM_AXES] = {
    /* LX     */ Action::GIMBAL_YAW,
    /* LY     */ Action::GIMBAL_PITCH,
    /* LT     */ Action::NONE,
    /* RX     */ Action::NONE,
    /* RY     */ Action::ZOOM,
    /* RT     */ Action::NONE,
    /* DPAD_X */ Action::NONE,
    /* DPAD_Y */ Action::NONE,
};


struct ActionName {
    Action      action;
    const char* name;
};

inline constexpr ActionName kActionNames[] = {
    { Action::NONE,          "NONE"          },
    { Action::GIMBAL_PITCH,  "GIMBAL_PITCH"  },
    { Action::GIMBAL_YAW,    "GIMBAL_YAW"    },
    { Action::GIMBAL_ROLL,   "GIMBAL_ROLL"   },
    { Action::ZOOM,          "ZOOM"          },
    { Action::GIMBAL_HOME,   "GIMBAL_HOME"   },
    { Action::GIMBAL_MODE,   "GIMBAL_MODE"   },
    { Action::IMAGE_CAPTURE, "IMAGE_CAPTURE" },
    { Action::VIDEO_RECORD,  "VIDEO_RECORD"  },
    { Action::CHANGE_VIEW,   "CHANGE_VIEW"   },
    { Action::CHANGE_RECORD, "CHANGE_RECORD" },
    { Action::TRACKING,      "TRACKING"      },

    { Action::ZOOM_TELE,     "ZOOM_TELE"     },
    { Action::ZOOM_WIDE,     "ZOOM_WIDE"     },
};


class GamepadController {
public:
    GamepadController();
    ~GamepadController();

    GamepadController(const GamepadController&) = delete;
    GamepadController& operator=(const GamepadController&) = delete;

    void setActionCallback(ActionCallback cb) { m_actionCallback = std::move(cb); }

    void setMappingFile(const std::string& path) { m_mappingFile = path; }

    void start();   
    void stop();    
    
    void update(int timeoutMs);

    int getAnalog(Action a) const { return m_analog[(int)a]; }

    bool quitRequested() const { return m_quit; }
    void requestQuit()         { m_quit = true; }   

    static bool        isAnalog(Action a);
    static const char* actionName(Action a);
    const char*        inputNameOf(Action a) const;   

private:
    // ---------- Thread GamepadReader ----------
    void onGamepadEvent(const GamepadEvent& ev);
    void handleButton(const GamepadEvent& ev);
    void handleAxis(const GamepadEvent& ev);
    void axisAsButton(int axis, bool pressedNow, Action a);
    void resetAnalog();
    void loadDefaultMap();

    // ---------- Thread main ----------
    void readConsole(int timeoutMs);
    void runActions();
    void handleCommand(const std::string& line);
    void setInputAction(const GamepadInput& in, Action a, bool fromFile = false);
    bool loadMapping();
    bool saveMapping() const;
    std::atomic<Action>& mapSlot(const GamepadInput& in);
    void printMap() const;
    static void printHelp();
    static void printList();
    static bool parseInput(const std::string& s, GamepadInput& out);
    static bool parseAction(const std::string& s, Action& out);
    static const char* inputName(const GamepadInput& in);

    static std::string toUpper(std::string s);


    GamepadReader        m_reader;
    ActionCallback       m_actionCallback;

    std::atomic<bool>    m_quit{false};
    std::atomic<bool>    m_logEvents{false};

    std::atomic<Action>  m_buttonMap[GAMEPAD_NUM_BUTTONS];
    std::atomic<Action>  m_axisMap[GAMEPAD_NUM_AXES];

    std::atomic<int>     m_request[(int)Action::COUNT] = {};  
    std::atomic<int>     m_analog[(int)Action::COUNT]  = {};   

    int                  m_lastReportedAnalog[(int)Action::COUNT] = {};
    std::string          m_consoleBuf;                            
    bool                 m_consoleOpen = true;
    std::string          m_mappingFile = DEFAULT_MAPPING_FILE;

    bool                 m_axisPressed[GAMEPAD_NUM_AXES] = {};
};

#endif