#include "gamepad_controller.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>
#include <poll.h>
#include <unistd.h>


GamepadController::GamepadController() {
    loadDefaultMap();
}

GamepadController::~GamepadController() {
    stop();
}

void GamepadController::start() {
    if (!loadMapping()) saveMapping();

    m_reader.setCallback([this](const GamepadEvent& ev) { onGamepadEvent(ev); });
    m_reader.start();

    printMap();
    printf("Waiting for gamepad... Type 'help' for commands (Ctrl+C to exit)\n");
    fflush(stdout);
}

void GamepadController::stop() {
    m_reader.stop();
}

void GamepadController::loadDefaultMap() {
    for (int i = 0; i < GAMEPAD_NUM_BUTTONS; ++i) m_buttonMap[i] = DEFAULT_BUTTON_MAP[i];
    for (int i = 0; i < GAMEPAD_NUM_AXES; ++i)    m_axisMap[i]   = DEFAULT_AXIS_MAP[i];
}

void GamepadController::update(int timeoutMs) {
    readConsole(timeoutMs);   // 1. keyboard 
    runActions();             // 2. callback for Actions
}

void GamepadController::runActions() {
    if (!m_actionCallback) return;

    for (int i = 0; i < (int)Action::COUNT; ++i) {
        Action a = (Action)i;

        if (isAnalog(a)) {
            int v = m_analog[i];
            if (v != m_lastReportedAnalog[i]) {
                m_lastReportedAnalog[i] = v;
                m_actionCallback(a);
            }
        } else {
            int n = m_request[i].exchange(0);  
            while (n-- > 0) m_actionCallback(a);
        }
    }
}

void GamepadController::onGamepadEvent(const GamepadEvent& ev) {
    switch (ev.type) {
    case GamepadEventType::Connected:
        printf("Connected: %s at %s\n", ev.deviceName.c_str(), ev.devicePath.c_str());
        break;

    case GamepadEventType::Disconnected:
        printf("Gamepad disconnected\n");
        resetAnalog();   
        break;

    case GamepadEventType::Button:
        if (m_logEvents)
            printf("Button %s %s\n", GamepadReader::buttonName(ev.number), ev.value ? "pressed" : "released");
        handleButton(ev);
        break;

    case GamepadEventType::Axis:
        if (m_logEvents)
            printf("Axis %s = %d\n", GamepadReader::axisName(ev.number), ev.value);
        handleAxis(ev);
        break;
    }
}

void GamepadController::handleButton(const GamepadEvent& ev) {
    if (ev.number < 0 || ev.number >= GAMEPAD_NUM_BUTTONS) return;

    Action a = m_buttonMap[ev.number];
    if (a == Action::NONE || isAnalog(a)) return;

    if (ev.value == 1) m_request[(int)a]++;
}

void GamepadController::axisAsButton(int axis, bool pressedNow, Action a) {
    if (pressedNow && !m_axisPressed[axis]) m_request[(int)a]++;
    m_axisPressed[axis] = pressedNow;
}

void GamepadController::handleAxis(const GamepadEvent& ev) {
    if (ev.number < 0 || ev.number >= GAMEPAD_NUM_AXES) return;

    Action a = m_axisMap[ev.number];
    if (a == Action::NONE) return;

    int v = std::clamp(ev.value, -32767, 32767);

    bool pressed = false;

    switch ((GamepadAxis)ev.number) {
    case GamepadAxis::DPAD_X:            // 0 or 32767
    case GamepadAxis::DPAD_Y:
        pressed = (v != 0);
        break;

    case GamepadAxis::LT:                // -32767..32767
    case GamepadAxis::RT:           
        pressed = (v > 0);
        break;

    default:                             // stick LX LY RX RY
        pressed = (std::abs(v) > 16000);
        break;
    }

    if (isAnalog(a)) m_analog[(int)a] = v;
    else             axisAsButton(ev.number, pressed, a);
}

void GamepadController::resetAnalog() {
    for (auto& v : m_analog) v = 0;
}

void GamepadController::readConsole(int timeoutMs) {
    if (!m_consoleOpen) {             
        usleep(timeoutMs * 1000);
        return;
    }

    pollfd p{STDIN_FILENO, POLLIN, 0};
    if (poll(&p, 1, timeoutMs) <= 0) return;   

    char tmp[256];
    ssize_t n = read(STDIN_FILENO, tmp, sizeof(tmp));
    if (n <= 0) { m_consoleOpen = false; return; }
    m_consoleBuf.append(tmp, (size_t)n);

    size_t pos;
    while ((pos = m_consoleBuf.find('\n')) != std::string::npos) {
        std::string line = m_consoleBuf.substr(0, pos);
        m_consoleBuf.erase(0, pos + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        handleCommand(line);
        if (m_quit) return;
        printf("> "); fflush(stdout);
    }
}

void GamepadController::handleCommand(const std::string& line) {
    std::istringstream ss(line);
    std::vector<std::string> t;
    for (std::string w; ss >> w;) t.push_back(toUpper(w));  
    if (t.empty()) return;

    const std::string& cmd = t[0];

    if (cmd == "HELP" || cmd == "H") {
        printHelp();
    }
    else if (cmd == "LIST") {
        printList();
    }
    else if (cmd == "SHOW") {
        printMap();
    }
    else if (cmd == "QUIT" || cmd == "Q") {
        m_quit = true;
    }
    else if (cmd == "LOG") {
        if (t.size() < 2) { printf("  Usage: log on|off\n"); return; }
        m_logEvents = (t[1] == "ON");
        printf("  Event log: %s\n", m_logEvents ? "ON" : "OFF");
    }
    else if (cmd == "SET") {
        GamepadInput in; Action a;
        if (t.size() < 3)          { printf("  Usage: set <BUTTON|AXIS> <ACTION>\n"); return; }
        if (!parseInput(t[1], in)) { printf("  '%s' is not a button or axis (type 'list')\n", t[1].c_str()); return; }
        if (!parseAction(t[2], a)) { printf("  '%s' is not an action (type 'list')\n", t[2].c_str()); return; }
        setInputAction(in, a);
    }
    else if (cmd == "CLEAR") {
        GamepadInput in;
        if (t.size() < 2 || !parseInput(t[1], in)) { 
            printf("  Usage: clear <BUTTON|AXIS>\n"); 
            return; 
        }
        setInputAction(in, Action::NONE);
    }
    else if (cmd == "RESET") {
        loadDefaultMap();
        resetAnalog();
        saveMapping();
        printf("  Restored default mapping\n");
        printMap();
    }
    else if (cmd == "LOAD") {
        loadDefaultMap();
        resetAnalog();
        loadMapping();
        printMap();
    }
    else {
        printf("  Unknown command: %s\n", cmd.c_str());
        printHelp();
    }
}

// Gan Action a cho dau vao in, cho cu dang giu Action do ve NONE.
// fromFile = true (khi doc file): chi gan, khong in, khong reset analog, khong luu file.
void GamepadController::setInputAction(const GamepadInput& in, Action a, bool fromFile) {
    if (in.isButton && isAnalog(a)) {
        printf("  %s is an analog action, it can only be set on an axis\n", actionName(a));
        return;
    }

    if (a != Action::NONE) {
        for (auto& s : m_buttonMap) if (s == a) s = Action::NONE;
        for (auto& s : m_axisMap)   if (s == a) s = Action::NONE;
    }

    mapSlot(in) = a;

    if (fromFile) return;  

    resetAnalog();

    printf("  %s -> %s\n", inputName(in), actionName(a));

    saveMapping();
}

bool GamepadController::saveMapping() const {
    const std::string tmpPath = m_mappingFile + ".tmp";
    std::ofstream f(tmpPath);
    if (!f) {
        printf("  WARNING: cannot write %s\n", tmpPath.c_str());
        return false;
    }

    for (int i = 0; i < GAMEPAD_NUM_BUTTONS; ++i)
        f << GamepadReader::buttonName(i) << " = " << actionName(m_buttonMap[i]) << "\n";
    f << "\n";   
    for (int i = 0; i < GAMEPAD_NUM_AXES; ++i)
        f << GamepadReader::axisName(i) << " = " << actionName(m_axisMap[i]) << "\n";
    f.close();

    if (!f || std::rename(tmpPath.c_str(), m_mappingFile.c_str()) != 0) {
        printf("  WARNING: cannot save %s\n", m_mappingFile.c_str());
        return false;
    }
    return true;
}

bool GamepadController::loadMapping() {
    std::ifstream f(m_mappingFile);
    if (!f) {
        printf("No mapping file %s, created it with the default mapping\n", m_mappingFile.c_str());
        return false;
    }

    std::string line;
    int  lineNo = 0;
    bool seen[(int)Action::COUNT] = {};   

    while (std::getline(f, line)) {
        ++lineNo;
        line = line.substr(0, line.find('#'));  

        std::replace(line.begin(), line.end(), '=', ' ');
        std::istringstream ss(line);
        std::string inName, actName;
        if (!(ss >> inName >> actName)) continue;   

        GamepadInput in; Action a;
        if (!parseInput(toUpper(inName), in) || !parseAction(toUpper(actName), a)) {
            printf("  WARNING: %s line %d: unknown name, skipped\n", m_mappingFile.c_str(), lineNo);
            continue;
        }
        if (in.isButton && isAnalog(a)) {
            printf("  WARNING: %s line %d: %s cannot be set on %s, skipped\n",
                   m_mappingFile.c_str(), lineNo, actionName(a), inputName(in));
            continue;
        }
        if (a != Action::NONE && seen[(int)a])
            printf("  WARNING: %s line %d: %s is used twice, only the last one (%s) is kept\n",
                   m_mappingFile.c_str(), lineNo, actionName(a), inputName(in));
        seen[(int)a] = true;

        setInputAction(in, a, true);
    }

    printf("Loaded mapping from %s\n", m_mappingFile.c_str());
    return true;
}

std::atomic<Action>& GamepadController::mapSlot(const GamepadInput& in) {
    return in.isButton ? m_buttonMap[in.index] : m_axisMap[in.index];
}

void GamepadController::printMap() const {
    printf("---- Button ----\n");
    for (int i = 0; i < GAMEPAD_NUM_BUTTONS; ++i) {
        Action a = m_buttonMap[i];
        if (a != Action::NONE) printf("  %-8s -> %s\n", GamepadReader::buttonName(i), actionName(a));
    }
    printf("---- Axis ----\n");
    for (int i = 0; i < GAMEPAD_NUM_AXES; ++i) {
        Action a = m_axisMap[i];
        if (a != Action::NONE) printf("  %-8s -> %s\n", GamepadReader::axisName(i), actionName(a));
    }
}

void GamepadController::printHelp() {
    printf(
"Commands:\n"
"  show                        Show current mapping\n"
"  list                        List button, axis and action names\n"
"  set <BUTTON|AXIS> <ACTION>  Set the action of a button/axis, e.g. set RX GIMBAL_YAW | set LB IMAGE_CAPTURE\n"
"  clear <BUTTON|AXIS>         Clear the action of a button/axis (same as set ... NONE)\n"
"  reset                       Restore default mapping (also saved to file)\n"
"  load                        Reload mapping from file (after editing it by hand)\n"
"  log on|off                  Print gamepad events on/off (to see button/axis names)\n"
"  quit                        Exit program\n");
}

void GamepadController::printList() {
    printf("Buttons: ");
    for (int i = 0; i < GAMEPAD_NUM_BUTTONS; ++i) printf("%s\t", GamepadReader::buttonName(i));
    printf("\nAxes   : ");
    for (int i = 0; i < GAMEPAD_NUM_AXES; ++i) printf("%s\t", GamepadReader::axisName(i));
    printf("\nAnalog actions (axis only):\n  ");
    for (const auto& n : kActionNames)
        if (isAnalog(n.action)) printf("%s\t", n.name);
    printf("\nOne-shot actions (button or axis):\n  ");
    for (const auto& n : kActionNames)
        if (!isAnalog(n.action) && n.action != Action::NONE) printf("%s\t", n.name);
    printf("\n");
}

bool GamepadController::parseInput(const std::string& u, GamepadInput& out) {
    for (int i = 0; i < GAMEPAD_NUM_BUTTONS; ++i)
        if (u == GamepadReader::buttonName(i)) { out = {true, i}; return true; }
    for (int i = 0; i < GAMEPAD_NUM_AXES; ++i)
        if (u == GamepadReader::axisName(i)) { out = {false, i}; return true; }
    return false;
}

bool GamepadController::parseAction(const std::string& u, Action& out) {
    for (const auto& n : kActionNames)
        if (u == n.name) { out = n.action; return true; }
    return false;
}

const char* GamepadController::inputName(const GamepadInput& in) {
    return in.isButton ? GamepadReader::buttonName(in.index) : GamepadReader::axisName(in.index);
}

const char* GamepadController::actionName(Action a) {
    for (const auto& n : kActionNames)
        if (n.action == a) return n.name;
    return "?";
}

const char* GamepadController::inputNameOf(Action a) const {
    for (int i = 0; i < GAMEPAD_NUM_BUTTONS; ++i)
        if (m_buttonMap[i] == a) return GamepadReader::buttonName(i);
    for (int i = 0; i < GAMEPAD_NUM_AXES; ++i)
        if (m_axisMap[i] == a) return GamepadReader::axisName(i);
    return "none";
}

bool GamepadController::isAnalog(Action a) {
    switch (a) {
    case Action::GIMBAL_PITCH:
    case Action::GIMBAL_YAW:
    case Action::GIMBAL_ROLL:
    case Action::ZOOM:
        return true;
    default:
        return false;
    }
}

std::string GamepadController::toUpper(std::string s) {
    for (auto& ch : s) ch = (char)std::toupper((unsigned char)ch);
    return s;
}