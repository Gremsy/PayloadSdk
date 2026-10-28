/**
 * This sample will update a package for the payload:
 *   1. Ask the user for the ssh password and the zip file
 *      (the payload ip is udp_ip_target in libs/payloadsdk.h)
 *   2. Upload the zip file to /home/guest/payload_update on the payload through ssh (sshpass), show the percent
 *      - upload to a temporary file <name>.zip.part
 *      - compare the md5 of the payload file with the local file
 *      - rename <name>.zip.part to <name>.zip and sync to the disk
 *   3. Send the command to reboot the payload. The payload unzips the package when it boots.
 *   4. Follow /var/log/apply_ota.log to show the percent of the update.
 *      When Apply OTA finishes, the payload reboots by itself again.
 *   5. The update is finished when the payload app sends MAVLink heartbeats again.
 *
 * Requirement on the host: sudo apt-get install sshpass openssh-client
 **/

#include "stdio.h"
#include "payloadSdkInterface.h"
#include "ssh_transfer.h"
#include "ota_monitor.h"
#include "terminal_ui.h"

#include <iostream>
#include <string>
#include <atomic>
#include <chrono>

// The ssh user of the payload
#define PAYLOAD_SSH_USER "guest"

// The folder on the payload to store the zip file
#define PAYLOAD_UPDATE_DIR "/home/guest/payload_update"

// Number of times to try the upload again if the network is lost
#define MAX_UPLOAD_RETRY 3

// Restart app id to reboot the payload (see payload_set_restart_apps.cpp)
#define REBOOT_PAYLOAD_APP_ID 5

// Max time to wait for the ACK of the reboot command
#define REBOOT_ACK_TIMEOUT_US 5000000

#define TOTAL_STEPS 5

// Redraw the upload progress at most every 200ms
#define UPLOAD_PROGRESS_INTERVAL_MS 200

PayloadSdkInterface* my_payload = nullptr;

// After the reboot command the payload stops sending, so sdkQuit() would wait forever:
// its read thread is blocked in recvfrom() and close() does not wake it up.
// The SDK threads are stopped when the program exits.
bool is_reboot_sent = false;

// Set by the SDK receive thread
std::atomic<bool> is_reboot_acked(false);
std::atomic<int> camera_heartbeat_count(0);     // heartbeats from the payload camera

void quit_handler(int sig);
void onPayloadStatusChanged(int event, double* param);
void onPayloadHeartbeat(mavlink_message_t msg);
bool isPayloadAppReady();

bool checkTools();
bool checkZipFile(const std::string& zip_path);
bool uploadPackage(const std::string& ip, const std::string& user, const std::string& zip_path, const std::string& remote_dir);
void onUploadProgress(long sent_bytes, long total_bytes);
void rebootPayload(const std::string& ip);

int main(int argc, char *argv[]){
	signal(SIGINT,quit_handler);
	signal(SIGPIPE, SIG_IGN);   // if ssh stops during the upload, report the error instead of being killed
	uiInit();

	// The payload ip is taken from udp_ip_target in libs/payloadsdk.h
	std::string payload_ip  = udp_ip_target;
	std::string user_name   = PAYLOAD_SSH_USER;
	std::string remote_dir  = PAYLOAD_UPDATE_DIR;

	uiTitle("Gremsy Payload Package Update");
	uiField("Payload IP", payload_ip);
	uiField("SSH user", user_name);
	uiField("Update folder", remote_dir);
	printf("\n");

	if(!checkTools())
		return 1;

	// get the information from the user
	std::string password    = readPassword("SSH password");
	std::string zip_path    = readInput("Package file (.zip)", (argc > 1) ? argv[1] : "");

	// 1. check the package
	uiStep(1, TOTAL_STEPS, "Checking the package");
	if(!checkZipFile(zip_path))
		return 1;

	// sshpass reads the password from the SSHPASS variable, so it is not shown in the process list
	setenv("SSHPASS", password.c_str(), 1);

	// 2. create the update folder (this also checks the ip and the password), then upload
	uiStep(2, TOTAL_STEPS, "Uploading the package to the payload");
	int result = runRemoteCommand(payload_ip, user_name, "mkdir -p " + remote_dir);
	if(result != SSH_RESULT_OK){
		uiFail(sshErrorText(result));
		uiFail("Can not log in to " + user_name + "@" + payload_ip);
		return 1;
	}
	uiOk("Logged in to " + user_name + "@" + payload_ip);

	if(!uploadPackage(payload_ip, user_name, zip_path, remote_dir)){
		uiFail("Upload failed, the payload is NOT rebooted");
		return 1;
	}

	// remember the boot id and the OTA log before rebooting, to know when a new boot / new OTA run starts
	T_PayloadSnapshot before_reboot;
	if(!readPayloadSnapshot(payload_ip, user_name, before_reboot)){
		uiFail("Can not read the state of the payload, the payload is NOT rebooted");
		return 1;
	}
	if(!before_reboot.log_readable)
		uiWarn("User " + user_name + " can not read the update log, the update status can not be followed");

	// 3. reboot the payload to apply the new package
	uiStep(3, TOTAL_STEPS, "Rebooting the payload");
	rebootPayload(payload_ip);

	// 4. follow Apply OTA until the payload reboots by itself again
	uiStep(4, TOTAL_STEPS, "Applying the update (do not power off the payload)");
	T_OtaProgress progress;
	E_OtaResult ota_result = monitorOtaUpdate(payload_ip, user_name, before_reboot, progress);

	// 5. the payload booted again: the update is finished when the payload app sends heartbeats
	if(ota_result == OTA_RESULT_SUCCESS){
		uiProgressEnd(true);
		uiStep(5, TOTAL_STEPS, "Starting the payload app");
		camera_heartbeat_count = 0; // only count the heartbeats of this boot
		ota_result = waitForPayloadApp(isPayloadAppReady, progress);
	}
	// show the stage where the update stopped, then mark it ✔ / ✖
	uiProgress(progress.percent, progress.stage, "");
	uiProgressEnd(ota_result == OTA_RESULT_SUCCESS);

	printOtaResult(ota_result, progress, getFileName(zip_path));

	return (ota_result == OTA_RESULT_SUCCESS) ? 0 : 1;
}

/**
 * Check the tools needed on the host
 **/
bool checkTools(){
	const char* tools[] = {"sshpass", "ssh", "md5sum"};
	for(const char* tool : tools){
		if(!isToolInstalled(tool)){
			uiFail(std::string("Can not find '") + tool + "'");
			uiInfo("Please run: sudo apt-get install sshpass openssh-client");
			return false;
		}
	}
	return true;
}

/**
 * Check the zip file before uploading
 **/
bool checkZipFile(const std::string& zip_path){
	if(zip_path.empty()){
		uiFail("The zip file is required");
		return false;
	}

	if(!isFileExist(zip_path)){
		uiFail("Can not find the file: " + zip_path);
		return false;
	}

	if(zip_path.size() < 4 || zip_path.substr(zip_path.size() - 4) != ".zip"){
		uiFail("The package must be a .zip file");
		return false;
	}

	if(getFileSize(zip_path) <= 0){
		uiFail("The zip file is empty");
		return false;
	}

	if(!isSafeFileName(getFileName(zip_path))){
		uiFail("The file name must only have a-z, A-Z, 0-9, '.', '_', '-' (no space)");
		return false;
	}

	uiOk(getFileName(zip_path) + "  (" + uiFormatSize(getFileSize(zip_path)) + ")");
	return true;
}

/**
 * Upload the zip file to <remote_dir>/<name>.zip
 * The payload only sees <name>.zip after the md5 is correct, so a broken upload is never unzipped.
 **/
bool uploadPackage(const std::string& ip, const std::string& user, const std::string& zip_path, const std::string& remote_dir){
	std::string remote_path = remote_dir + "/" + getFileName(zip_path);
	std::string part_path   = remote_path + ".part";

	std::string local_md5 = getLocalMd5(zip_path);
	if(local_md5.empty()){
		uiFail("Can not read the md5 of " + zip_path);
		return false;
	}

	for(int attempt = 1; attempt <= MAX_UPLOAD_RETRY; attempt++){
		if(attempt > 1)
			uiInfo("Uploading again  (attempt " + std::to_string(attempt) + "/" + std::to_string(MAX_UPLOAD_RETRY) + ")");

		// copy to the temporary file, onUploadProgress() shows the percent
		auto start = std::chrono::steady_clock::now();
		int result = copyFileToPayload(ip, user, zip_path, part_path, onUploadProgress);
		long upload_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
		uiProgressClear();
		if(result != SSH_RESULT_OK){
			uiWarn(sshErrorText(result));

			// do not try again if the password is wrong or the user pressed Ctrl+C
			if(result == SSH_RESULT_WRONG_PASSWORD
				|| result == SSH_RESULT_HOST_KEY_CHANGED
				|| result == SSH_RESULT_INTERRUPTED)
				return false;

			usleep(2000000);
			continue;
		}

		// compare the md5 of the file on the payload with the local file
		std::string remote_md5 = readRemoteMd5(ip, user, part_path);
		if(remote_md5 != local_md5){
			uiWarn("The uploaded file is broken (md5 is not matched), uploading again");
			runRemoteCommand(ip, user, "rm -f " + part_path);
			usleep(2000000);
			continue;
		}

		// rename to the real name and write the data to the disk before rebooting
		result = runRemoteCommand(ip, user, "mv -f " + part_path + " " + remote_path + " && sync");
		if(result != SSH_RESULT_OK){
			uiWarn(sshErrorText(result));
			usleep(2000000);
			continue;
		}

		long size = getFileSize(zip_path);
		long speed = (upload_ms > 0) ? (long)(size * 1000.0 / upload_ms) : size;
		uiOk("Uploaded " + uiFormatSize(size) + " in " + std::to_string((upload_ms + 500) / 1000) + "s  ("
			+ uiFormatSize(speed) + "/s), md5 verified");
		return true;
	}

	// try to remove the unfinished file (may fail if the network is still lost)
	runRemoteCommand(ip, user, "rm -f " + part_path);
	return false;
}

/**
 * Show the upload: percent, size, speed and remaining time
 **/
void onUploadProgress(long sent_bytes, long total_bytes){
	static auto start = std::chrono::steady_clock::now();
	static auto last_draw = std::chrono::steady_clock::now();
	auto now = std::chrono::steady_clock::now();

	// a new upload (or a new attempt) starts from 0
	if(sent_bytes <= UPLOAD_BLOCK_SIZE)
		start = now;

	long since_draw_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_draw).count();
	if(since_draw_ms < UPLOAD_PROGRESS_INTERVAL_MS && sent_bytes < total_bytes)
		return;
	last_draw = now;

	double seconds = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
	double speed = (seconds > 0) ? sent_bytes / seconds : 0;     // bytes per second
	int percent = (total_bytes > 0) ? (int)(sent_bytes * 100 / total_bytes) : 100;

	// "12.1/38.1 MB · 19.3 MB/s · ETA 00:13"
	char sizes[64];
	snprintf(sizes, sizeof(sizes), "%.1f/%.1f MB", sent_bytes / 1048576.0, total_bytes / 1048576.0);
	std::string detail = std::string(sizes) + " · " + uiFormatSize((long)speed) + "/s";
	if(speed > 0 && sent_bytes < total_bytes){
		long left_s = (long)((total_bytes - sent_bytes) / speed);
		char eta[32];
		snprintf(eta, sizeof(eta), " · ETA %02ld:%02ld", left_s / 60, left_s % 60);
		detail += eta;
	}

	uiProgress(percent, "Uploading", detail);
}

/**
 * Connect to the payload by PayloadSdk and send the reboot command
 **/
void rebootPayload(const std::string& ip){
	T_ConnInfo s_conn = {
		CONTROL_UDP,
		(char*)ip.c_str(),
		udp_port_target
	};

	uiInfo("Connecting to the payload app (MAVLink) ...");

	// the SDK prints its debug log, hide it
	uiMuteOutput();

	// creat payloadsdk object
	my_payload = new PayloadSdkInterface(s_conn);

	// init payload
	my_payload->sdkInitConnection();

	// register callback function
	my_payload->regPayloadStatusChanged(onPayloadStatusChanged);
	my_payload->regPayloadHeartbeatChanged(onPayloadHeartbeat);

	// check connection
	my_payload->checkPayloadConnection();

	uiRestoreStdout();
	uiOk("Connected to the payload app");

	// send command to reboot the payload (set the flag first: the ACK may come back immediately)
	is_reboot_sent = true;
	my_payload->setPayloadRestartApps(REBOOT_PAYLOAD_APP_ID);

	// wait for the ACK
	for(int waited_us = 0; waited_us < REBOOT_ACK_TIMEOUT_US && !is_reboot_acked; waited_us += 100000)
		usleep(100000);
	if(!is_reboot_acked)
		uiWarn("No answer for the reboot command, checking if the payload reboots anyway");

	// do not call sdkQuit() here, see is_reboot_sent
}

void quit_handler( int sig ){
    uiRestoreStdout();
    uiProgressEnd(false);
    uiWarn("Stopped by the user (Ctrl+C)");

    // close payload interface
    try {
        if(my_payload != nullptr && !is_reboot_sent)
            my_payload->sdkQuit();
    }
    catch (int error){}

    // end program here
    exit(0);
}

void onPayloadStatusChanged(int event, double* param){
	switch(event){
	case PAYLOAD_ACK:{
		if(param[0] == MAV_CMD_USER_4 && is_reboot_sent && !is_reboot_acked){
			// param[0]: command
			// param[1]: result
			// param[2]: progress
			is_reboot_acked = true;
			if(param[1] == MAV_RESULT_ACCEPTED)
				uiOk("Reboot command accepted");
			else
				uiWarn("Reboot command answered with result " + std::to_string((int)param[1]));
		}
		break;
	}
	default: break;
	}
}

/**
 * The camera component is created by the payload app, so its heartbeat means the payload app is running
 **/
void onPayloadHeartbeat(mavlink_message_t msg){
	if(msg.compid >= MAV_COMP_ID_CAMERA && msg.compid <= MAV_COMP_ID_CAMERA6)
		camera_heartbeat_count++;
}

bool isPayloadAppReady(){
	return camera_heartbeat_count > 0;
}
