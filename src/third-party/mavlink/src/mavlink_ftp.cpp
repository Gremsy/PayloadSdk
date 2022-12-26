
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>

#include "mavlink_ftp.h"

MavlinkFTP::MavlinkFTP()
{
	// initialize session
}

MavlinkFTP::~MavlinkFTP()
{
}

void 
MavlinkFTP::
setMessageQueue(std::queue<mavlink_file_transfer_protocol_t>& m_queue){
	m_local_queue = m_queue;
}
void
MavlinkFTP::
handle_message(const mavlink_message_t *msg)
{
	printf("MavlinkFTP::handle_message\n");

	if (msg->msgid == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
		mavlink_file_transfer_protocol_t ftp_request;
		mavlink_msg_file_transfer_protocol_decode(msg, &ftp_request);
		_process_request(&ftp_request, msg->sysid, msg->compid);
	}
}

/// @brief Processes an FTP message
void
MavlinkFTP::
_process_request(
	mavlink_file_transfer_protocol_t *ftp_req,
	uint8_t target_system_id,
	uint8_t target_comp_id)
{
	bool stream_send = false;
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

	ErrorCode errorCode = kErrNone;

	// basic sanity checks; must validate length before use
	if (payload->size > kMaxDataLength) {
		errorCode = kErrInvalidDataSize;
		goto out;
	}

	// check the sequence number: if this is a resent request, resend the last response
	if (_last_reply_valid) {
		mavlink_file_transfer_protocol_t *last_reply = reinterpret_cast<mavlink_file_transfer_protocol_t *>(_last_reply);
		PayloadHeader *last_payload = reinterpret_cast<PayloadHeader *>(&last_reply->payload[0]);

		if (payload->seq_number + 1 == last_payload->seq_number) {
			// this is the same request as the one we replied to last. It means the (n)ack got lost, and the GCS
			// resent the request
			return;
		}
	}

	switch (payload->opcode) {
	case kCmdNone:
		break;

	case kCmdTerminateSession:
		// errorCode = _workTerminate(payload);
		break;

	case kCmdResetSessions:
		// errorCode = _workReset(payload);
		break;

	case kCmdListDirectory:
		// errorCode = _workList(payload);
		break;

	case kCmdOpenFileRO:
		errorCode = _workOpen(payload, O_RDONLY);
		break;

	case kCmdCreateFile:
		// errorCode = _workOpen(payload, O_CREAT | O_TRUNC | O_WRONLY);
		break;

	case kCmdOpenFileWO:
		// errorCode = _workOpen(payload, O_CREAT | O_WRONLY);
		break;

	case kCmdReadFile:
		errorCode = _workRead(payload);
		break;

	case kCmdBurstReadFile:
		// errorCode = _workBurst(payload, target_system_id, target_comp_id);
		stream_send = true;
		break;

	case kCmdWriteFile:
		// errorCode = _workWrite(payload);
		break;

	case kCmdRemoveFile:
		// errorCode = _workRemoveFile(payload);
		break;

	case kCmdRename:
		// errorCode = _workRename(payload);
		break;

	case kCmdTruncateFile:
		// errorCode = _workTruncateFile(payload);
		break;

	case kCmdCreateDirectory:
		// errorCode = _workCreateDirectory(payload);
		break;

	case kCmdRemoveDirectory:
		// errorCode = _workRemoveDirectory(payload);
		break;

	case kCmdCalcFileCRC32:
		// errorCode = _workCalcFileCRC32(payload);
		break;

	default:
		errorCode = kErrUnknownCommand;
		break;
	}

 out:
	payload->seq_number++;

	// handle success vs. error
	if (errorCode == kErrNone) {
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspAck;

	} else {
		int r_errno = errno;
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspNak;
		payload->size = 1;

		if (r_errno == EEXIST) {
			errorCode = kErrFailFileExists;

		} else if (r_errno == ENOENT && errorCode == kErrFailErrno) {
			errorCode = kErrFileNotFound;
		}

		payload->data[0] = errorCode;

		if (errorCode == kErrFailErrno) {
			payload->size = 2;
			payload->data[1] = r_errno;
		}
	}

	_last_reply_valid = false;

	// Stream download replies are sent through mavlink stream mechanism. Unless we need to Nack.
	if (!stream_send || errorCode != kErrNone) {
		// respond to the request
		ftp_req->target_system = target_system_id;
		ftp_req->target_network = 0;
		ftp_req->target_component = target_comp_id;
		_reply(ftp_req);
	}
}

/// @brief Sends the specified FTP response message out through mavlink
void
MavlinkFTP::
_reply(mavlink_file_transfer_protocol_t *ftp_req)
{
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

	// keep a copy of the last sent response ((n)ack), so that if it gets lost and the GCS resends the request,
	// we can simply resend the response.
	// we only keep small responses to reduce RAM usage and avoid large memcpy's. The larger responses are all data
	// retrievals without side-effects, meaning it's ok to reexecute them if a response gets lost
	if (payload->size <= sizeof(uint32_t)) {
		_last_reply_valid = true;
		memcpy(_last_reply, ftp_req, sizeof(_last_reply));
	}

	// mavlink_msg_file_transfer_protocol_send_struct(_mavlink->get_channel(), ftp_req);

	m_local_queue.push(*ftp_req);
}

/// @brief Responds to an Open command
MavlinkFTP::ErrorCode
MavlinkFTP::_workOpen(PayloadHeader *payload, int oflag)
{
// 	if (_session_info.fd >= 0) {
// 		printf("FTP: Open failed - out of sessions\n");
// 		return kErrNoSessionsAvailable;
// 	}

// 	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
// 	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);

// #ifdef MAVLINK_FTP_DEBUG
// 	printf("FTP: open '%s'", _work_buffer1);
// #endif

// 	uint32_t fileSize = 0;
// 	struct stat st;

// 	if (stat(_work_buffer1, &st) != 0) {
// 		// fail only if requested open for read
// 		if (oflag & O_RDONLY) {
// 			return kErrFailErrno;

// 		} else {
// 			st.st_size = 0;
// 		}
// 	}

// 	fileSize = st.st_size;

// 	// Set mode to 666 incase oflag has O_CREAT
// 	int fd = ::open(_work_buffer1, oflag, PX4_O_MODE_666);

// 	if (fd < 0) {
// 		return kErrFailErrno;
// 	}

// 	_session_info.fd = fd;
// 	_session_info.file_size = fileSize;
// 	_session_info.stream_download = false;

// 	payload->session = 0;
// 	payload->size = sizeof(uint32_t);
// 	std::memcpy(payload->data, &fileSize, payload->size);

	return kErrNone;
}

/// @brief Responds to a Read command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRead(PayloadHeader *payload)
{
// 	if (payload->session != 0 || _session_info.fd < 0) {
// 		return kErrInvalidSession;
// 	}

// #ifdef MAVLINK_FTP_DEBUG
// 	printf("FTP: read offset:%" PRIu32, payload->offset);
// #endif

// 	// We have to test seek past EOF ourselves, lseek will allow seek past EOF
// 	if (payload->offset >= _session_info.file_size) {
// 		printf("request past EOF");
// 		return kErrEOF;
// 	}

// 	if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
// 		printf("seek fail");
// 		return kErrFailErrno;
// 	}

// 	int bytes_read = ::read(_session_info.fd, &payload->data[0], kMaxDataLength);

// 	if (bytes_read < 0) {
// 		// Negative return indicates error other than eof
// 		printf("read fail %d", bytes_read);
// 		return kErrFailErrno;
// 	}

// 	payload->size = bytes_read;

	return kErrNone;
}