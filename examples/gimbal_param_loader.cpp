/**
 * @file    gimbal_param_loader.cpp
 * @brief   Sample: load a gTune XML parameter file onto a gimbal and verify it.
 *
 * What this sample does
 * ---------------------
 *   1. Parses a gTune parameter file (XML).
 *   2. Translates each gTune parameter name into the firmware parameter ID
 *      used over MAVLink (see kParamMap below).
 *   3. For every mapped parameter:
 *        a. writes the value to the gimbal,
 *        b. reads it back,
 *        c. compares the read-back value with the expected one,
 *        d. retries on timeout or mismatch.
 *   4. Prints a per-parameter report and a final summary.
 *
 * Usage
 * -----
 *   gimbal_param_loader <file.xml> [-v]
 *
 *     <file.xml>   gTune parameter file to load
 *     -v           verbose: print every PARAM_VALUE message received
 *
 * Exit codes
 * ----------
 *   0   every parameter was written and verified
 *   1   at least one parameter failed to write or verify
 *   2   invalid arguments or unusable input file
 *
 * Notes
 * -----
 *   - Parameters present in the XML but absent from kParamMap are skipped
 *     (never sent to the gimbal) and listed in the output.
 */

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include <unistd.h>

#include "payloadSdkInterface.h"

// ============================================================================
// Connection settings (same as the SDK samples)
// ============================================================================
#if (CONTROL_METHOD == CONTROL_UART)
T_ConnInfo s_conn = {CONTROL_UART, payload_uart_port, payload_uart_baud};
#else
T_ConnInfo s_conn = {CONTROL_UDP, udp_ip_target, udp_port_target};
#endif

PayloadSdkInterface* my_payload = nullptr;

// SDK callback, defined near the end of this file.
void onPayloadParamChanged(int event, char* param_char, double* param);

namespace {

// ============================================================================
// Tunable settings
// ============================================================================
constexpr int    kMaxAttempts       = 5;      ///< Write/read-back attempts per parameter
constexpr int    kReplyTimeoutMs    = 300;    ///< Max wait for a PARAM_VALUE reply
constexpr int    kWriteSettleMs     = 100;    ///< Pause between a write and its read-back
constexpr int    kConnectSettleUs   = 500000; ///< Pause after connecting
constexpr int    kDisconnectWaitUs  = 1000000;///< Pause before closing the link
constexpr float  kTolerance         = 1e-3f;  ///< Float comparison tolerance
constexpr size_t kMaxParamIdLength  = 15;     ///< MAVLink param_id is 16 bytes incl. '\0'

bool g_verbose = false;

// ============================================================================
// Parameter name mapping: gTune name -> firmware parameter ID
// ============================================================================
struct ParamMapping {
    const char* gtuneName;   ///< Name as it appears in the gTune XML file
    const char* firmwareId;  ///< Parameter ID used over MAVLink
};

/**
 * Parameters that this sample is allowed to write.
 * Any gTune parameter not listed here is never sent to the gimbal.
 */
const ParamMapping kParamMap[] = {
    {"PITCH_P",         "STIFF_TILT"},
    {"ROLL_P",          "STIFF_ROLL"},
    {"YAW_P",           "STIFF_PAN"},
    {"PITCH_POWER",     "PWR_TILT"},
    {"ROLL_POWER",      "PWR_ROLL"},
    {"YAW_POWER",       "PWR_PAN"},
    {"YAW_I",           "FILTER_OUT"},
    {"ROLL_FILTER",     "MAPPING_ANGLE"},
    {"GYRO_LPF",        "GYRO_LPF"},
    {"PITCH_FOLLOW",    "FLW_SP_TILT"},
    {"YAW_FOLLOW",      "FLW_SP_PAN"},
    {"PITCH_FILTER",    "FLW_LPF_TILT"},
    {"YAW_FILTER",      "FLW_LPF_PAN"},
    {"TILT_WINDOW",     "FLW_WD_TILT"},
    {"PAN_WINDOW",      "FLW_WD_PAN"},
    {"TRAVEL_MIN_PIT",  "RC_LIM_MIN_TILT"},
    {"TRAVEL_MAX_PIT",  "RC_LIM_MAX_TILT"},
    {"TRAVEL_MIN_ROLL", "RC_LIM_MIN_ROLL"},
    {"TRAVEL_MAX_ROLL", "RC_LIM_MAX_ROLL"},
    {"RC_ROLL_LPF",     "RC_LPF_ROLL"},
    {"RC_PITCH_TRIM",   "RC_TRIM_TILT"},
    {"RC_ROLL_TRIM",    "RC_TRIM_ROLL"},
    {"RC_ROLL_MODE",    "RC_MODE_ROLL"},
    {"JOY_AXIS",        "RC_REVERSE_AXIS"},
    {"BAUDRATE_COM2",   "BAUDRATE_COM2"},
    {"BAUDRATE_COM4",   "BAUDRATE_COM4"},
    {"RADIO_TYPE",      "RC_TYPE"},
    {"DIR_MOTOR_PITCH", "ENABLE_ADRC"}
};

/// One parameter ready to be sent to the gimbal.
struct Param {
    std::string gtuneName;
    std::string firmwareId;
    float       value = 0.0f;
};

// ============================================================================
// XML loading
// ============================================================================

/**
 * Reads a gTune XML file and returns the parameters that have a mapping.
 *
 * The file is expected to be flat, one entry per parameter:
 *     <Params Index="PITCH_P" Value="50"/>
 *
 * @return true if at least one parameter was mapped.
 */
bool loadParams(const std::string& path, std::vector<Param>& out)
{
    std::ifstream file(path);
    if (!file) {
        std::cerr << "Cannot open file: " << path << "\n";
        return false;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    const std::string text = buffer.str();

    std::map<std::string, std::string> lookup;
    for (const auto& m : kParamMap) lookup[m.gtuneName] = m.firmwareId;

    const std::regex entry(
        R"re(<Params\s+Index\s*=\s*"([^"]+)"\s+Value\s*=\s*"([^"]+)")re");

    size_t total = 0;
    std::vector<std::string> skipped;

    for (auto it = std::sregex_iterator(text.begin(), text.end(), entry);
         it != std::sregex_iterator(); ++it) {
        ++total;

        Param p;
        p.gtuneName = (*it)[1].str();
        p.value     = std::strtof((*it)[2].str().c_str(), nullptr);

        const auto found = lookup.find(p.gtuneName);
        if (found == lookup.end()) {
            skipped.push_back(p.gtuneName);
            continue;
        }
        p.firmwareId = found->second;
        out.push_back(p);
    }

    if (total == 0) {
        std::cerr << "No parameters found in XML file.\n";
        return false;
    }

    std::cout << "XML: " << total << " parameters, " << out.size()
            << " mapped, " << skipped.size() << " skipped.\n";

    if (!skipped.empty()) {
        std::cout << "Skipped (not writable: not in allowed parameter list):\n";
        for (const auto& name : skipped) std::cout << "  SKIP " << name << "\n";
    }

    return !out.empty();
}

// ============================================================================
// Reply tracking
//
// The SDK delivers PARAM_VALUE messages on its own reader thread through a
// callback. AckTracker hands those values over to the main thread safely.
// ============================================================================
class AckTracker {
public:
    void store(const std::string& id, float value)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            values_[id] = value;
        }
        cv_.notify_all();
    }

    void clear(const std::string& id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        values_.erase(id);
    }

    /// Waits for a reply for `id`. Returns false on timeout.
    bool waitFor(const std::string& id, float& value)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const bool arrived = cv_.wait_for(
            lock, std::chrono::milliseconds(kReplyTimeoutMs),
            [&] { return values_.count(id) > 0; });
        if (arrived) value = values_[id];
        return arrived;
    }

private:
    std::mutex                   mutex_;
    std::condition_variable      cv_;
    std::map<std::string, float> values_;
};

AckTracker g_acks;

// ============================================================================
// Gimbal connection
// ============================================================================

/// Ctrl+C handler: close the SDK link and exit.
void onInterrupt(int)
{
    printf("\nTERMINATING AT USER REQUEST\n");
    try { if (my_payload) my_payload->sdkQuit(); } catch (int) {}
    exit(0);
}

void connectPayload()
{
    signal(SIGINT, onInterrupt);

    my_payload = new PayloadSdkInterface(s_conn);
    my_payload->sdkInitConnection();

    printf("Waiting for payload signal!\n");
    my_payload->regPayloadParamChanged(onPayloadParamChanged);
    my_payload->checkPayloadConnection();
    usleep(kConnectSettleUs);
}

void disconnectPayload()
{
    usleep(kDisconnectWaitUs);
    try { my_payload->sdkQuit(); } catch (int) {}
}

// ============================================================================
// Write + verify one parameter
// ============================================================================

/**
 * Writes `p` to the gimbal, reads it back and compares.
 *
 * @param[out] actual    Last value read from the gimbal (NaN if none).
 * @param[out] attempts  Number of attempts used.
 * @return true if the gimbal reports the expected value.
 */
bool writeAndVerify(const Param& p, float& actual, int& attempts)
{
    char id[kMaxParamIdLength + 2] = {0};
    std::strncpy(id, p.firmwareId.c_str(), kMaxParamIdLength);

    for (attempts = 1; attempts <= kMaxAttempts; ++attempts) {
        my_payload->setPayloadGimbalParamByID(id, p.value);
        usleep(kWriteSettleMs * 1000);

        g_acks.clear(p.firmwareId);
        my_payload->getPayloadGimbalSettingByID(id);
        if (!g_acks.waitFor(p.firmwareId, actual)) continue;   // timeout: retry

        if (std::fabs(actual - p.value) <= kTolerance) return true;
    }

    attempts = kMaxAttempts;
    return false;
}

// ============================================================================
// Command line
// ============================================================================
bool parseArgs(int argc, char** argv, std::string& xmlPath)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-v")            g_verbose = true;
        else if (arg[0] != '-')     xmlPath = arg;
        else {
            std::cerr << "Unknown option: " << arg << "\n";
            return false;
        }
    }
    if (xmlPath.empty()) {
        std::cerr << "Usage: " << argv[0] << " <file.xml> [-v]\n";
        return false;
    }
    return true;
}

}  // namespace

void onPayloadParamChanged(int event, char* param_char, double* param)
{
    if (event != PAYLOAD_GB_PARAMS) return;

    if (g_verbose)
        printf("[callback] event=%d id=%.16s index=%.0f value=%.3f\n",
               event, param_char, param[0], param[1]);

    char id[17] = {0};
    std::memcpy(id, param_char, 16);

    g_acks.store(id, static_cast<float>(param[1]));
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv)
{
    std::string xmlPath;
    if (!parseArgs(argc, argv, xmlPath)) return 2;

    std::vector<Param> params;
    if (!loadParams(xmlPath, params)) return 2;

    connectPayload();

    std::cout << "\n" << std::left
              << std::setw(18) << "PARAM"
              << std::setw(12) << "EXPECTED"
              << std::setw(12) << "ACTUAL"
              << "RESULT\n";

    int passed = 0, failed = 0;

    for (const auto& p : params) {
        float actual   = NAN;
        int   attempts = 0;
        const bool ok = writeAndVerify(p, actual, attempts);

        std::cout << std::left
                  << std::setw(18) << p.firmwareId
                  << std::setw(12) << p.value;
        if (std::isnan(actual)) std::cout << std::setw(12) << "-";
        else                    std::cout << std::setw(12) << actual;
        std::cout << (ok ? "OK" : "FAIL") << " (attempt " << attempts << ")\n";

        ok ? ++passed : ++failed;
    }

    std::cout << "\nSummary: " << passed << " OK, " << failed << " FAIL, "
              << params.size() << " total\n";

    disconnectPayload();
    return failed == 0 ? 0 : 1;
}