#pragma once
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "payloadSdkInterface.h"

struct GpsFix
{
    double lat, lon, alt;   // degrees, degrees, meters
};

inline bool toDouble(const std::string &s, double &v)
{
    char *end = nullptr;
    v = std::strtod(s.c_str(), &end);
    return end != s.c_str() && *end == '\0' && std::isfinite(v);
}

inline bool validFix(const GpsFix &f, std::string &err)
{
    if (std::fabs(f.lat) > 90.0)           { err = "lat must be within -90..90"; return false; }
    if (std::fabs(f.lon) > 180.0)          { err = "lon must be within -180..180"; return false; }
    if (f.alt < -100.0 || f.alt > 10000.0) { err = "alt must be within -100..10000 m"; return false; }
    return true;
}

// Accepts "lat lon [alt]" and/or "--lat X --lon Y --alt Z" (any subset).
inline bool parseGpsArgs(const std::vector<std::string> &t, GpsFix &f, std::string &err)
{
    GpsFix n = f;
    double *slot[3] = {&n.lat, &n.lon, &n.alt};
    bool set[3] = {false, false, false};
    double pos[3];
    size_t np = 0;

    for (size_t i = 0; i < t.size(); ++i)
    {
        double v;
        if (t[i].rfind("--", 0) == 0)
        {
            if (i + 1 >= t.size() || !toDouble(t[i + 1], v))
            {
                err = "missing or invalid value after " + t[i];
                return false;
            }
            int k = t[i] == "--lat" ? 0 : t[i] == "--lon" ? 1 : t[i] == "--alt" ? 2 : -1;
            if (k < 0)
            {
                err = "unknown option " + t[i];
                return false;
            }
            *slot[k] = v;
            set[k] = true;
            ++i;
        }
        else
        {
            if (np >= 3 || !toDouble(t[i], v))
            {
                err = "invalid argument: " + t[i];
                return false;
            }
            pos[np++] = v;
        }
    }
    for (size_t k = 0; k < np; ++k)
        if (!set[k])
            *slot[k] = pos[k];

    if (!validFix(n, err))
        return false;
    f = n;
    return true;
}

class GpsSimulator
{
public:
    explicit GpsSimulator(const GpsFix &init) : fix_(init) {}
    ~GpsSimulator() { stop(); }

    GpsSimulator(const GpsSimulator &) = delete;
    GpsSimulator &operator=(const GpsSimulator &) = delete;

    void start()
    {
        if (running_.exchange(true))
            return;
        th_ = std::thread([this] {
            payload_ = new PayloadSdkInterface(conn_);
            payload_->sdkInitConnection();
            printf("[gps] waiting for payload signal...\n");
            payload_->checkPayloadConnection();
            printf("[gps] payload connected, sending simulated GPS\n");
            connected_ = true;   // set after the message so the console help prints below it
            loop();
        });
    }

    void stop()
    {
        if (!running_.exchange(false))
            return;
        if (th_.joinable())
        {
            if (connected_)
                th_.join();
            else
                th_.detach();
        }
        if (payload_ && connected_)
        {
            try { payload_->sdkQuit(); } catch (...) {}
        }
    }

    bool isConnected() const { return connected_.load(); }

    void set(const GpsFix &f)
    {
        std::lock_guard<std::mutex> l(m_);
        fix_ = f;
    }

    GpsFix get() const
    {
        std::lock_guard<std::mutex> l(m_);
        return fix_;
    }

private:
    void loop()
    {
        int cnt = 0;
        uint32_t boot_ms = 0;
        while (running_)
        {
            const GpsFix f = get();
            const bool fixed = cnt >= kNoFixMessages;

            mavlink_gps_raw_int_t raw = {};
            raw.time_usec = std::chrono::duration_cast<std::chrono::microseconds>(
                                std::chrono::system_clock::now().time_since_epoch()).count();
            raw.fix_type = fixed ? GPS_FIX_TYPE_DGPS : GPS_FIX_TYPE_NO_FIX;
            raw.lat = static_cast<int32_t>(std::llround(f.lat * 1e7));
            raw.lon = static_cast<int32_t>(std::llround(f.lon * 1e7));
            raw.alt = static_cast<int32_t>(std::llround(f.alt * 1e3));
            raw.eph = 100;
            raw.epv = 150;
            raw.vel = 0;
            raw.satellites_visible = fixed ? 8 : 3;
            raw.alt_ellipsoid = raw.alt;
            raw.h_acc = 2000;
            raw.v_acc = 3000;
            raw.vel_acc = 100;
            raw.hdg_acc = 500;
            payload_->sendPayloadGPSRawInt(raw);

            if (fixed)
            {
                mavlink_global_position_int_t pos = {};
                pos.time_boot_ms = boot_ms;
                pos.lat = raw.lat;
                pos.lon = raw.lon;
                pos.alt = raw.alt;
                pos.relative_alt = raw.alt;   // the entered altitude is used as height above ground
                payload_->sendPayloadGPSPosition(pos);
            }
            ++cnt;
            boot_ms += 100;
            usleep(100000);   // 10 Hz
        }
    }

    static constexpr int kNoFixMessages = 20;

#if (CONTROL_METHOD == CONTROL_UART)
    T_ConnInfo conn_ = {CONTROL_UART, payload_uart_port, payload_uart_baud};
#else
    T_ConnInfo conn_ = {CONTROL_UDP, udp_ip_target, udp_port_target};
#endif
    PayloadSdkInterface *payload_ = nullptr;
    mutable std::mutex m_;
    GpsFix fix_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread th_;
};

inline void printGpsHelp()
{
    printf("\n"
           "GPS simulator commands:\n"
           "  gps <lat> <lon> [alt]                set position        e.g. gps 10.83 106.71 30\n"
           "  gps --lat <v> --lon <v> --alt <v>    set chosen fields   e.g. gps --alt 50\n"
           "  gps                                  print current position\n"
           "  help                                 show this help\n"
           "  quit                                 exit\n"
           "Units: lat/lon in degrees, alt in meters\n\n");
}

inline void printGpsFix(const GpsSimulator &sim)
{
    const GpsFix f = sim.get();
    printf("[gps] lat=%.7f lon=%.7f alt=%.1f m\n", f.lat, f.lon, f.alt);
}

// Blocking command loop, meant to run in its own thread.
// Waits for the payload connection before printing the help / accepting commands.
inline void runConsole(GpsSimulator *sim, std::atomic<bool> *quit)
{
    while (!sim->isConnected() && !*quit)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (*quit)
        return;

    printGpsHelp();
    printGpsFix(*sim);

    std::string line;
    while (!*quit && std::getline(std::cin, line))
    {
        std::istringstream iss(line);
        std::vector<std::string> tok{std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>()};
        if (tok.empty())
            continue;
        const std::string cmd = tok[0];
        tok.erase(tok.begin());

        if (cmd == "gps")
        {
            GpsFix f = sim->get();
            std::string err;
            if (tok.empty())
                printGpsFix(*sim);
            else if (!parseGpsArgs(tok, f, err))
                printf("[gps] error: %s\n", err.c_str());
            else
            {
                sim->set(f);
                printGpsFix(*sim);
            }
        }
        else if (cmd == "help")
            printGpsHelp();
        else if (cmd == "quit" || cmd == "q")
            *quit = true;
        else
            printf("[gps] unknown command '%s' (type 'help')\n", cmd.c_str());
    }
}