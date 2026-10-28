#include "ssh_transfer.h"
#include "terminal_ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <termios.h>
#include <sys/stat.h>
#include <sys/wait.h>

/**
 * Put the text inside single quotes so the shell does not split it
 **/
static std::string shellQuote(const std::string& text){
	std::string result = "'";
	for(char c : text){
		if(c == '\'')
			result += "'\\''";
		else
			result += c;
	}
	result += "'";
	return result;
}

/**
 * Run a shell command and return its exit code (SSH_RESULT_INTERRUPTED if it was killed by a signal)
 **/
static int runCommand(const std::string& command){
	fflush(stdout); // print our messages before the output of the command
	int status = system(command.c_str());

	if(status == -1)
		return SSH_RESULT_INTERRUPTED;
	if(WIFSIGNALED(status))
		return SSH_RESULT_INTERRUPTED;
	return WEXITSTATUS(status);
}

/**
 * Run a shell command and return the first word of its output (empty if failed)
 **/
static std::string readCommandOutput(const std::string& command){
	fflush(stdout);
	FILE* pipe = popen(command.c_str(), "r");
	if(pipe == nullptr)
		return "";

	char buffer[128] = {0};
	std::string output;
	if(fscanf(pipe, "%127s", buffer) == 1)
		output = buffer;

	int status = pclose(pipe);
	if(status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0)
		return "";
	return output;
}

/**
 * Print the label and read one line from the keyboard.
 * Return default_value if the user only press Enter.
 **/
std::string readInput(const std::string& label, const std::string& default_value){
	uiPrompt(label, default_value);

	std::string value;
	std::getline(std::cin, value);

	// remove spaces and quotes around the text (added when a file is pasted or dragged into the terminal)
	const char* trim_chars = " \t\r\n'\"";
	size_t start = value.find_first_not_of(trim_chars);
	if(start == std::string::npos)
		return default_value;
	size_t end = value.find_last_not_of(trim_chars);
	return value.substr(start, end - start + 1);
}

/**
 * Read the password without showing it on the terminal
 **/
std::string readPassword(const std::string& label){
	uiPrompt(label, "");

	// turn off echo
	struct termios old_term, new_term;
	tcgetattr(STDIN_FILENO, &old_term);
	new_term = old_term;
	new_term.c_lflag &= ~ECHO;
	tcsetattr(STDIN_FILENO, TCSANOW, &new_term);

	std::string value;
	std::getline(std::cin, value);

	// turn on echo again
	tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
	printf("\n");

	return value;
}

bool isToolInstalled(const std::string& tool_name){
	std::string command = "command -v " + tool_name + " > /dev/null 2>&1";
	return runCommand(command) == 0;
}

bool isFileExist(const std::string& path){
	struct stat info;
	return (stat(path.c_str(), &info) == 0) && S_ISREG(info.st_mode);
}

long getFileSize(const std::string& path){
	struct stat info;
	if(stat(path.c_str(), &info) != 0)
		return -1;
	return info.st_size;
}

/**
 * Get "package.zip" from "/home/user/package.zip"
 **/
std::string getFileName(const std::string& path){
	size_t pos = path.find_last_of('/');
	if(pos == std::string::npos)
		return path;
	return path.substr(pos + 1);
}

/**
 * Only allow a-z, A-Z, 0-9, '.', '_', '-' so the name is the same on the host and the payload shell
 **/
bool isSafeFileName(const std::string& file_name){
	if(file_name.empty())
		return false;

	for(char c : file_name){
		bool is_safe = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9')
					|| c == '.' || c == '_' || c == '-';
		if(!is_safe)
			return false;
	}
	return true;
}

std::string getLocalMd5(const std::string& path){
	return readCommandOutput("md5sum " + shellQuote(path));
}

/**
 * Run a command on the payload by ssh, return the exit code
 **/
int runRemoteCommand(const std::string& ip, const std::string& user, const std::string& remote_command){
	std::string command = std::string("sshpass -e ssh ") + SSH_OPTIONS
						+ " " + user + "@" + ip
						+ " " + shellQuote(remote_command) + " 2>/dev/null";
	return runCommand(command);
}

/**
 * Read the md5 of a file on the payload (empty if failed)
 **/
std::string readRemoteMd5(const std::string& ip, const std::string& user, const std::string& remote_path){
	std::string command = std::string("sshpass -e ssh ") + SSH_OPTIONS
						+ " " + user + "@" + ip
						+ " " + shellQuote("md5sum " + remote_path) + " 2>/dev/null";
	return readCommandOutput(command);
}

/**
 * Run a command on the payload and read all of its output, return the exit code.
 * connect_timeout_s is put before SSH_OPTIONS because ssh uses the first value of an option.
 **/
int readRemoteOutput(const std::string& ip, const std::string& user, const std::string& remote_command,
                     int connect_timeout_s, std::string& output){
	std::string command = std::string("sshpass -e ssh -o ConnectTimeout=") + std::to_string(connect_timeout_s)
						+ " " + SSH_OPTIONS
						+ " " + user + "@" + ip
						+ " " + shellQuote(remote_command) + " 2>/dev/null";

	output.clear();
	fflush(stdout);
	FILE* pipe = popen(command.c_str(), "r");
	if(pipe == nullptr)
		return SSH_RESULT_INTERRUPTED;

	char buffer[4096];
	size_t count;
	while((count = fread(buffer, 1, sizeof(buffer), pipe)) > 0)
		output.append(buffer, count);

	int status = pclose(pipe);
	if(status == -1 || !WIFEXITED(status))
		return SSH_RESULT_INTERRUPTED;
	return WEXITSTATUS(status);
}

/**
 * Copy the local file to <user>@<ip>:<remote_path>, return the exit code.
 * The file is written into the stdin of "ssh ... cat > <remote_path>", so the sent bytes can be counted
 * and on_progress(sent_bytes, total_bytes) is called after each block.
 * SIGPIPE must be ignored by the caller: if ssh stops, fwrite() fails instead of killing the program.
 **/
int copyFileToPayload(const std::string& ip, const std::string& user, const std::string& local_path,
                      const std::string& remote_path, void (*on_progress)(long sent_bytes, long total_bytes)){
	FILE* file = fopen(local_path.c_str(), "rb");
	if(file == nullptr)
		return SSH_RESULT_INTERRUPTED;
	long total_bytes = getFileSize(local_path);

	std::string command = std::string("sshpass -e ssh ") + SSH_OPTIONS
						+ " " + user + "@" + ip
						+ " " + shellQuote("cat > " + shellQuote(remote_path)) + " 2>/dev/null";

	fflush(stdout);
	FILE* pipe = popen(command.c_str(), "w");
	if(pipe == nullptr){
		fclose(file);
		return SSH_RESULT_INTERRUPTED;
	}

	static char buffer[UPLOAD_BLOCK_SIZE];
	long sent_bytes = 0;
	size_t count;
	while((count = fread(buffer, 1, sizeof(buffer), file)) > 0){
		if(fwrite(buffer, 1, count, pipe) != count)
			break;  // ssh stopped (wrong password, network lost), pclose() gives the reason

		sent_bytes += count;
		if(on_progress != nullptr)
			on_progress(sent_bytes, total_bytes);
	}
	fclose(file);

	// wait for ssh to send the last data and close the remote file
	int status = pclose(pipe);
	if(status == -1 || !WIFEXITED(status))
		return SSH_RESULT_INTERRUPTED;
	if(WEXITSTATUS(status) == SSH_RESULT_OK && sent_bytes != total_bytes)
		return SSH_RESULT_INTERRUPTED;
	return WEXITSTATUS(status);
}

std::string sshErrorText(int result){
	switch(result){
	case SSH_RESULT_OK:
		return "";
	case SSH_RESULT_WRONG_PASSWORD:
		return "Wrong ssh password";
	case SSH_RESULT_HOST_KEY_CHANGED:
		return "The host key of the payload is unknown";
	case SSH_RESULT_INTERRUPTED:
		return "The command was interrupted";
	case SSH_RESULT_CONNECT_ERROR:
		return "Can not connect to the payload, please check the network and the ip";
	default:
		return "The command returned code " + std::to_string(result) + " (network lost, disk full or permission denied)";
	}
}
