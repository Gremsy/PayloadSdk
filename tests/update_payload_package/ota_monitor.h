/**
 * Follow the "Apply OTA" process of the payload after it reboots:
 *   1. Wait for the payload to reboot (boot id changes)
 *   2. Read /var/log/apply_ota.log by ssh and estimate the percent from its content
 *   3. When Apply OTA finishes, the payload reboots again by itself -> wait for it -> done
 **/

#ifndef OTA_MONITOR_H
#define OTA_MONITOR_H

#include <string>
#include <vector>

// Files on the payload
#define OTA_LOG_PATH                "/var/log/apply_ota.log"
#define BOOT_ID_PATH                "/proc/sys/kernel/random/boot_id"    // changes after every boot

// Timing
#define OTA_POLL_INTERVAL_US        1000000     // check the payload every 1s
#define OTA_CONNECT_TIMEOUT_S       3           // short ssh timeout to check the payload quickly
#define WAIT_REBOOT_TIMEOUT_S       180         // max time for the payload to reboot and come back
#define OTA_STALL_TIMEOUT_S         180         // max time without any new line in the OTA log
#define PAYLOAD_APP_TIMEOUT_S       180         // max time for the payload app to send heartbeats after the last boot

typedef enum {
	OTA_RESULT_SUCCESS = 0,
	OTA_RESULT_NO_PACKAGE,          // Apply OTA did not find the package
	OTA_RESULT_INSTALL_FAILED,      // a component did not install successfully
	OTA_RESULT_NOT_REBOOTED,        // the payload did not reboot / did not come back
	OTA_RESULT_LOG_STALLED,         // the OTA log stopped changing
	OTA_RESULT_LOG_NOT_READABLE,    // the ssh user can not read the OTA log
	OTA_RESULT_UNKNOWN,             // the payload rebooted but the OTA log of this update is lost
	OTA_RESULT_APP_NOT_STARTED,     // the update is applied but the payload app does not send heartbeats
} E_OtaResult;

// What the payload looks like at one moment
typedef struct {
	std::string boot_id;
	bool log_readable;
	std::vector<std::string> runs;  // the last "Start Apply OTA" runs in the log, oldest first
} T_PayloadSnapshot;

// What we understand from one "Start Apply OTA" run
typedef struct {
	int percent;
	std::string stage;
	std::string detail;             // extra information of the stage, e.g. "flash 63%"
	std::string package_name;       // package that Apply OTA found
	std::string version_info;       // from the "Package_Info written" line
	bool no_package;                // "Payload file not found ... after N attempts"
	bool finished;                  // the lock file is deleted = Apply OTA ends
	std::vector<std::string> installing;    // components with "Found do_install.sh"
	std::vector<std::string> installed;     // components with "Successfully executed do_install.sh"
	std::vector<std::string> skipped;       // components skipped on purpose (e.g. gimbal FW not selected)
	std::vector<std::string> failed;        // components with "Failed to execute do_install.sh"
	std::vector<std::string> error_lines;
} T_OtaProgress;

bool readPayloadSnapshot(const std::string& ip, const std::string& user, T_PayloadSnapshot& snapshot);
T_OtaProgress parseOtaRun(const std::string& run);
std::vector<std::string> getFailedComponents(const T_OtaProgress& progress);

/**
 * Follow the update after the reboot command was sent.
 * before_reboot: snapshot taken just before sending the reboot command
 **/
E_OtaResult monitorOtaUpdate(const std::string& ip, const std::string& user,
                             const T_PayloadSnapshot& before_reboot, T_OtaProgress& progress);

/**
 * After monitorOtaUpdate() returns OTA_RESULT_SUCCESS (payload booted again, 98%):
 * wait until is_payload_app_ready() returns true -> 100%
 **/
E_OtaResult waitForPayloadApp(bool (*is_payload_app_ready)(), T_OtaProgress& progress);

void printOtaResult(E_OtaResult result, const T_OtaProgress& progress, const std::string& uploaded_package);

#endif // OTA_MONITOR_H
