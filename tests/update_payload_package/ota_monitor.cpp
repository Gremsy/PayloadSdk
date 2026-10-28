#include "ota_monitor.h"
#include "ssh_transfer.h"
#include "terminal_ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <sstream>

// One step of Apply OTA: when the keyword is in the log, the update is at least <percent>
typedef struct {
	const char* keyword;
	int percent;
	const char* stage;
} T_OtaStage;

// Percent of each step, estimated from a real OrusL update log (about 90s in total)
static const T_OtaStage OTA_STAGES[] = {
	{"Start Apply OTA",                                  2,  "Apply OTA started"},
	{"Checking /sdcard directory",                       3,  "Searching for the package"},
	{"Payload found in OTA directory",                   5,  "Package found"},
	{"ZIP metadata is valid",                            8,  "Package metadata is valid"},
	{"Decryption complete",                             16,  "Decryption complete"},
	{"[INFO] Extracting",                               18,  "Extracting the package"},
	{"Processing gremsy-app-setting directory",         22,  "Installing gremsy-app-setting"},
	{"/gimbal-official-firmwares/",                     25,  "Updating gimbal firmware"},
	{"Firmware update successful",                      55,  "Gimbal firmware updated"},
	{"/gremsy-app-lt6911-update/",                      58,  "Installing gremsy-app-lt6911-update"},
	{"/gremsy-app-mavlink-debugger/",                   61,  "Installing gremsy-app-mavlink-debugger"},
	{"/gremsy-app-password-manager/",                   64,  "Installing gremsy-app-password-manager"},
	{"/gremsy-app-payload/",                            68,  "Installing gremsy-app-payload"},
	{"/gremsy-app-tracking/",                           74,  "Installing gremsy-app-tracking"},
	{"/gremsy-app-videostream-ethernet/",               78,  "Installing gremsy-app-videostream-ethernet"},
	{"/gremsy-app-videostream-hdmi/",                   82,  "Installing gremsy-app-videostream-hdmi"},
	{"/gremsy-stm32-fw-update/",                        85,  "Installing gremsy-stm32-fw-update"},
	{"Running create_db.py",                            88,  "Updating the database"},
	{"create_db.py completed successfully",             93,  "Database updated"},
	{"Deleting files in",                               95,  "Cleaning up"},
	{"do_apply_ota.lock has been successfully deleted", 97,  "Apply OTA finished"},
};

// The gimbal firmware flashing takes about 65s, so it moves the percent from 25 to 55
#define GIMBAL_FLASH_START_PERCENT  25
#define GIMBAL_FLASH_END_PERCENT    55

#define RUN_SEPARATOR "@@RUN@@\n"

// A component prints one of these lines when it skips itself on purpose.
// Its do_install.sh then returns an error ("Failed to execute ..."), which is expected.
static const char* SKIP_KEYWORDS[] = {
	"Skipping firmware update as expected",
	"Gimbal FW update not selected. Skip",
};

static long secondsSince(const std::chrono::steady_clock::time_point& start){
	return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count();
}

static bool contains(const std::string& text, const std::string& keyword){
	return text.find(keyword) != std::string::npos;
}

/**
 * Get "gremsy-app-tracking" from "/tmp/temp_update/gremsy-app-tracking/" or "gremsy-app-setting"
 **/
static std::string getComponentName(std::string path){
	while(!path.empty() && (path.back() == '/' || path.back() == ' '))
		path.pop_back();
	size_t pos = path.find_last_of('/');
	if(pos != std::string::npos)
		path = path.substr(pos + 1);
	return path;
}

/**
 * Get the text between <begin> and <end> in the line (empty if not found)
 **/
static std::string getTextBetween(const std::string& line, const std::string& begin, const std::string& end){
	size_t start = line.find(begin);
	if(start == std::string::npos)
		return "";
	start += begin.size();
	size_t stop = line.find(end, start);
	if(stop == std::string::npos)
		return "";
	return line.substr(start, stop - start);
}

/**
 * The first line with a time stamp identifies a run of Apply OTA
 **/
static std::string getRunFingerprint(const std::string& run){
	std::istringstream stream(run);
	std::string line;
	while(std::getline(stream, line)){
		if(!line.empty() && line[0] >= '0' && line[0] <= '9')
			return line;
	}
	return "";
}

bool readPayloadSnapshot(const std::string& ip, const std::string& user, T_PayloadSnapshot& snapshot){
	// print the boot id, then the last 3 "Start Apply OTA" runs of the log, each run starts with RUN_SEPARATOR
	std::string remote_command =
		std::string("echo \"BOOT_ID=$(cat ") + BOOT_ID_PATH + ")\"; "
		"LOG=" + OTA_LOG_PATH + "; "
		"if [ ! -e $LOG ]; then echo LOG=MISSING; "
		"elif [ ! -r $LOG ]; then echo LOG=NO_ACCESS; "
		"else echo LOG=OK; "
		"awk '/Start Apply OTA/ { n++; if (n > 3) delete run[n-3] } "
		"n > 0 { run[n] = run[n] $0 \"\\n\" } "
		"END { for (i = (n > 3 ? n - 2 : 1); i <= n; i++) printf \"@@RUN@@\\n%s\", run[i] }' $LOG; fi";

	std::string output;
	int result = readRemoteOutput(ip, user, remote_command, OTA_CONNECT_TIMEOUT_S, output);
	if(result != SSH_RESULT_OK)
		return false;

	snapshot.boot_id = getTextBetween(output, "BOOT_ID=", "\n");
	snapshot.log_readable = !contains(output, "LOG=NO_ACCESS");
	snapshot.runs.clear();

	size_t pos = output.find(RUN_SEPARATOR);
	while(pos != std::string::npos){
		size_t start = pos + std::string(RUN_SEPARATOR).size();
		size_t next = output.find(RUN_SEPARATOR, start);
		snapshot.runs.push_back(output.substr(start, (next == std::string::npos) ? std::string::npos : next - start));
		pos = next;
	}

	return !snapshot.boot_id.empty();
}

T_OtaProgress parseOtaRun(const std::string& run){
	T_OtaProgress progress;
	progress.percent = 0;
	progress.no_package = false;
	progress.finished = false;

	bool flashing_gimbal = false;
	std::string current_component = "";

	// the flashing tool updates its percent with '\r', so split by '\r' too
	std::string text = run;
	std::replace(text.begin(), text.end(), '\r', '\n');

	std::istringstream stream(text);
	std::string line;
	while(std::getline(stream, line)){
		// known steps
		for(const T_OtaStage& stage : OTA_STAGES){
			if(contains(line, stage.keyword) && stage.percent >= progress.percent){
				progress.percent = stage.percent;
				progress.stage = stage.stage;
				progress.detail = "";
			}
		}

		// unknown component: show its name, keep the percent
		if(contains(line, "Processing directory:")){
			std::string name = getComponentName(line.substr(line.find("Processing directory:") + 21));
			progress.stage = "Installing " + name;
			progress.detail = "";
			flashing_gimbal = (name == "gimbal-official-firmwares");
		}

		// gimbal firmware flashing: "[FLASH] Write percent ==> 63.21 [%]"
		if(flashing_gimbal && contains(line, "Write percent ==>")){
			float flash = atof(line.substr(line.find("==>") + 3).c_str());
			int percent = GIMBAL_FLASH_START_PERCENT
						+ (int)(flash * (GIMBAL_FLASH_END_PERCENT - GIMBAL_FLASH_START_PERCENT) / 100);
			if(percent >= progress.percent){
				progress.percent = percent;
				progress.stage = "Updating gimbal firmware";
				progress.detail = "flash " + std::to_string((int)flash) + "%";
			}
		}
		if(contains(line, "Firmware update successful")){
			flashing_gimbal = false;
			progress.detail = "";
		}

		// components
		if(contains(line, "Found do_install.sh in")){
			current_component = getComponentName(getTextBetween(line, "Found do_install.sh in ", ". Running"));
			progress.installing.push_back(current_component);
		}
		if(contains(line, "Successfully executed do_install.sh in"))
			progress.installed.push_back(getComponentName(line.substr(line.find("do_install.sh in ") + 17)));
		if(contains(line, "Failed to execute do_install.sh in"))
			progress.failed.push_back(getComponentName(line.substr(line.find("do_install.sh in ") + 17)));

		for(const char* keyword : SKIP_KEYWORDS){
			if(contains(line, keyword) && !current_component.empty()
				&& std::find(progress.skipped.begin(), progress.skipped.end(), current_component) == progress.skipped.end()){
				progress.skipped.push_back(current_component);
				progress.stage = "Skipping " + current_component + " (not selected)";
				progress.detail = "";
				flashing_gimbal = false;
				if(current_component == "gimbal-official-firmwares" && progress.percent < GIMBAL_FLASH_END_PERCENT)
					progress.percent = GIMBAL_FLASH_END_PERCENT;
			}
		}

		// package information
		if(contains(line, "Payload found in OTA directory") && progress.package_name.empty())
			progress.package_name = getTextBetween(line, "(", ")");
		// "Package_Info written to ...: version='v307.59.12', build_date='2026-10-26', payload='Payload-ORUSL'."
		if(contains(line, "Package_Info written to"))
			progress.version_info = getTextBetween(line, "version='", "'")
								+ "  (build " + getTextBetween(line, "build_date='", "'")
								+ ", " + getTextBetween(line, "payload='", "'") + ")";

		// end of Apply OTA
		if(contains(line, "Payload file not found in /sdcard after"))
			progress.no_package = true;
		if(contains(line, "do_apply_ota.lock has been successfully deleted"))
			progress.finished = true;

		if(contains(line, "[ERROR]") || contains(line, "Error:"))
			progress.error_lines.push_back(line);
	}

	return progress;
}

std::vector<std::string> getFailedComponents(const T_OtaProgress& progress){
	auto has = [](const std::vector<std::string>& list, const std::string& name){
		return std::find(list.begin(), list.end(), name) != list.end();
	};

	// failed = started but not installed, or "Failed to execute", except the components skipped on purpose
	std::vector<std::string> failed;
	for(const std::string& name : progress.installing){
		bool is_failed = !has(progress.installed, name) || has(progress.failed, name);
		if(is_failed && !has(progress.skipped, name) && !has(failed, name))
			failed.push_back(name);
	}
	return failed;
}

/**
 * Wait until the payload answers with a boot id different from old_boot_id
 **/
static bool waitForNewBoot(const std::string& ip, const std::string& user, const std::string& old_boot_id,
                           int percent, const std::string& stage, T_PayloadSnapshot& snapshot){
	auto start = std::chrono::steady_clock::now();

	while(secondsSince(start) < WAIT_REBOOT_TIMEOUT_S){
		bool online = readPayloadSnapshot(ip, user, snapshot);

		if(online && snapshot.boot_id != old_boot_id)
			return true;

		uiProgress(percent, stage, online ? "still online" : "offline");
		usleep(OTA_POLL_INTERVAL_US);
	}
	return false;
}

/**
 * The runs written after the run <old_fingerprint> (the last run before our reboot).
 * If the old run is not in the list (log was cleared), all runs are new.
 **/
static std::vector<std::string> getNewRuns(const std::vector<std::string>& runs, const std::string& old_fingerprint){
	size_t first_new = 0;
	for(size_t i = 0; i < runs.size(); i++){
		if(!old_fingerprint.empty() && getRunFingerprint(runs[i]) == old_fingerprint)
			first_new = i + 1;
	}

	std::vector<std::string> new_runs;
	for(size_t i = first_new; i < runs.size(); i++){
		if(!getRunFingerprint(runs[i]).empty())
			new_runs.push_back(runs[i]);
	}
	return new_runs;
}

E_OtaResult monitorOtaUpdate(const std::string& ip, const std::string& user,
                             const T_PayloadSnapshot& before_reboot, T_OtaProgress& progress){
	progress = parseOtaRun("");
	std::string old_fingerprint = before_reboot.runs.empty() ? "" : getRunFingerprint(before_reboot.runs.back());

	// 1. wait for the reboot that we requested
	progress.stage = "Rebooting the payload";
	T_PayloadSnapshot snapshot;
	if(!waitForNewBoot(ip, user, before_reboot.boot_id, 0, progress.stage, snapshot))
		return OTA_RESULT_NOT_REBOOTED;

	std::string update_boot_id = snapshot.boot_id;
	std::string update_fingerprint = "";    // the run of this update, known when it starts

	// 2. follow the OTA log until Apply OTA finishes
	progress.percent = 1;
	progress.stage = "Starting Apply OTA";
	auto last_change = std::chrono::steady_clock::now();
	int last_percent = progress.percent;
	std::string last_text = "";

	while(true){
		if(secondsSince(last_change) > OTA_STALL_TIMEOUT_S)
			return OTA_RESULT_LOG_STALLED;

		if(!readPayloadSnapshot(ip, user, snapshot)){
			// offline: the payload may be rebooting at the end of Apply OTA, keep checking
			uiProgress(progress.percent, progress.stage, "payload offline");
			usleep(OTA_POLL_INTERVAL_US);
			continue;
		}

		if(!snapshot.log_readable)
			return OTA_RESULT_LOG_NOT_READABLE;

		bool rebooted_again = (snapshot.boot_id != update_boot_id);
		std::vector<std::string> new_runs = getNewRuns(snapshot.runs, old_fingerprint);

		// the run of this update is the first new run after our reboot
		std::string update_run = "";
		if(update_fingerprint.empty() && !new_runs.empty())
			update_fingerprint = getRunFingerprint(new_runs[0]);
		for(const std::string& run : snapshot.runs){
			if(!update_fingerprint.empty() && getRunFingerprint(run) == update_fingerprint)
				update_run = run;
		}

		if(!update_run.empty()){
			progress = parseOtaRun(update_run);

			// never go back
			if(progress.percent < last_percent)
				progress.percent = last_percent;

			// remember when the log changed last time (to detect a stalled update)
			std::string text = std::to_string(progress.percent) + progress.stage + progress.detail;
			if(text != last_text){
				last_text = text;
				last_percent = progress.percent;
				last_change = std::chrono::steady_clock::now();
			}

			if(progress.no_package)
				return OTA_RESULT_NO_PACKAGE;

			if(progress.finished){
				if(!getFailedComponents(progress).empty())
					return OTA_RESULT_INSTALL_FAILED;

				// the payload already rebooted after Apply OTA (a newer run exists or the boot id changed)
				if(rebooted_again || new_runs.size() >= 2){
					progress.percent = 98;
					progress.stage = "Rebooting the payload (automatic)";
					uiProgress(progress.percent, progress.stage, "");
					return OTA_RESULT_SUCCESS;
				}
				break;
			}
		}

		// rebooted before the end of Apply OTA, or the log of this update is lost
		if(rebooted_again)
			return OTA_RESULT_UNKNOWN;

		uiProgress(progress.percent, progress.stage, progress.detail);
		usleep(OTA_POLL_INTERVAL_US);
	}

	// 3. Apply OTA reboots the payload by itself, wait for it
	uiProgress(progress.percent, progress.stage, "");
	progress.stage = "Rebooting the payload (automatic)";
	if(!waitForNewBoot(ip, user, update_boot_id, progress.percent, progress.stage, snapshot))
		return OTA_RESULT_NOT_REBOOTED;

	progress.percent = 98;
	uiProgress(progress.percent, progress.stage, "");
	return OTA_RESULT_SUCCESS;
}

E_OtaResult waitForPayloadApp(bool (*is_payload_app_ready)(), T_OtaProgress& progress){
	auto start = std::chrono::steady_clock::now();
	progress.stage = "Starting the payload app";

	while(secondsSince(start) < PAYLOAD_APP_TIMEOUT_S){
		if(is_payload_app_ready()){
			progress.percent = 100;
			uiProgress(progress.percent, progress.stage, "heartbeat received");
			return OTA_RESULT_SUCCESS;
		}

		uiProgress(progress.percent, progress.stage, "no heartbeat yet");
		usleep(500000);
	}
	return OTA_RESULT_APP_NOT_STARTED;
}

void printOtaResult(E_OtaResult result, const T_OtaProgress& progress, const std::string& uploaded_package){
	const char* result_text[] = {
		"UPDATE SUCCESSFUL",
		"UPDATE FAILED - Apply OTA did not find the package",
		"UPDATE FAILED - some components were not installed",
		"UPDATE FAILED - the payload did not reboot or did not come back online",
		"UPDATE FAILED - the OTA log stopped changing",
		"UPDATE STATUS UNKNOWN - can not read the update log",
		"UPDATE STATUS UNKNOWN - the payload rebooted before Apply OTA finished",
		"UPDATE FAILED - the payload app does not answer (no heartbeat)",
	};

	std::vector<std::pair<std::string, std::string>> fields;
	fields.push_back({"Package", uploaded_package});
	if(!progress.version_info.empty())
		fields.push_back({"Version", progress.version_info});

	std::string components = std::to_string(progress.installed.size()) + "/"
							+ std::to_string(progress.installing.size()) + " installed";
	for(const std::string& name : progress.skipped)
		components += ", " + name + " skipped";
	fields.push_back({"Components", components});

	if(result != OTA_RESULT_SUCCESS)
		fields.push_back({"Stopped at", std::to_string(progress.percent) + "% " + progress.stage});

	std::vector<std::string> notes;
	if(!progress.package_name.empty() && progress.package_name != uploaded_package)
		notes.push_back("Warning: the payload applied " + progress.package_name);

	if(result != OTA_RESULT_SUCCESS){
		for(const std::string& name : getFailedComponents(progress))
			notes.push_back("Not finished: " + name);
		for(const std::string& line : progress.error_lines)
			notes.push_back("Log: " + line);
	}

	uiResult(result == OTA_RESULT_SUCCESS, result_text[result], fields, notes);
}
