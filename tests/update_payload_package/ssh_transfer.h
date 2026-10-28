/**
 * Helper functions to copy a file to the payload through ssh (sshpass).
 * The ssh password must be set in the SSHPASS environment variable before calling them.
 **/

#ifndef SSH_TRANSFER_H
#define SSH_TRANSFER_H

#include <string>

// Skip the host key check because the payload ip may be reused by different devices.
// ServerAlive: stop the command after 15s if the network is lost (cable unplugged, payload rebooted)
#define SSH_OPTIONS "-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o ConnectTimeout=10 " \
                    "-o ServerAliveInterval=5 -o ServerAliveCountMax=3 -o LogLevel=ERROR"

// Size of one block written to ssh when uploading
#define UPLOAD_BLOCK_SIZE           (64 * 1024)

// Exit codes from sshpass / ssh
#define SSH_RESULT_OK               0
#define SSH_RESULT_WRONG_PASSWORD   5     // sshpass: invalid/incorrect password
#define SSH_RESULT_HOST_KEY_CHANGED 6     // sshpass: host public key is unknown
#define SSH_RESULT_CONNECT_ERROR    255   // ssh: can not connect to the payload
#define SSH_RESULT_INTERRUPTED      -1    // the command was stopped by a signal (Ctrl+C)

/**
 * Read input from the keyboard
 **/
std::string readInput(const std::string& label, const std::string& default_value);
std::string readPassword(const std::string& label);

/**
 * Check the local file
 **/
bool isToolInstalled(const std::string& tool_name);
bool isFileExist(const std::string& path);
long getFileSize(const std::string& path);
std::string getFileName(const std::string& path);
bool isSafeFileName(const std::string& file_name);
std::string getLocalMd5(const std::string& path);

/**
 * Work with the payload
 **/
int runRemoteCommand(const std::string& ip, const std::string& user, const std::string& remote_command);
std::string readRemoteMd5(const std::string& ip, const std::string& user, const std::string& remote_path);
int readRemoteOutput(const std::string& ip, const std::string& user, const std::string& remote_command,
                     int connect_timeout_s, std::string& output);
int copyFileToPayload(const std::string& ip, const std::string& user, const std::string& local_path,
                      const std::string& remote_path, void (*on_progress)(long sent_bytes, long total_bytes));

/**
 * The meaning of an exit code from sshpass / ssh / scp
 **/
std::string sshErrorText(int result);

#endif // SSH_TRANSFER_H
