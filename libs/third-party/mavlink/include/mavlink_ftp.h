
#pragma once
#include <dirent.h>
#include <queue>

#include <common/mavlink.h>

/// MAVLink remote file server. Support FTP like commands using MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL message.
class MavlinkFTP
{
public:
	MavlinkFTP();
	~MavlinkFTP();

	void setMessageQueue(std::queue<mavlink_file_transfer_protocol_t>& m_queue);
	/// Handle possible FTP message
	void handle_message(const mavlink_message_t *msg);

	/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
	/// This needs to be packed, because it's typecasted from mavlink_file_transfer_protocol_t.payload, which starts
	/// at a 3 byte offset, causing an unaligned access to seq_number and offset
	struct __attribute__((__packed__)) PayloadHeader {
		uint16_t	seq_number;	///< sequence number for message
		uint8_t		session;	///< Session id for read and write commands
		uint8_t		opcode;		///< Command opcode
		uint8_t		size;		///< Size of data
		uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
		uint8_t		burst_complete; ///< Only used if req_opcode=kCmdBurstReadFile - 1: set of burst packets complete, 0: More burst packets coming.
		uint8_t		padding;        ///< 32 bit aligment padding
		uint32_t	offset;		///< Offsets for List and Read commands
		uint8_t		data[];		///< command data, varies by Opcode
	};

	/// @brief Command opcodes
	enum Opcode : uint8_t {
		kCmdNone,		///< ignored, always acked
		kCmdTerminateSession,	///< Terminates open Read session
		kCmdResetSessions,	///< Terminates all open Read sessions
		kCmdListDirectory,	///< List files in <path> from <offset>
		kCmdOpenFileRO,		///< Opens file at <path> for reading, returns <session>
		kCmdReadFile,		///< Reads <size> bytes from <offset> in <session>
		kCmdCreateFile,		///< Creates file at <path> for writing, returns <session>
		kCmdWriteFile,		///< Writes <size> bytes to <offset> in <session>
		kCmdRemoveFile,		///< Remove file at <path>
		kCmdCreateDirectory,	///< Creates directory at <path>
		kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty
		kCmdOpenFileWO,		///< Opens file at <path> for writing, returns <session>
		kCmdTruncateFile,	///< Truncate file at <path> to <offset> length
		kCmdRename,		///< Rename <path1> to <path2>
		kCmdCalcFileCRC32,	///< Calculate CRC32 for file at <path>
		kCmdBurstReadFile,	///< Burst download session file

		kRspAck = 128,		///< Ack response
		kRspNak			///< Nak response
	};

	/// @brief Error codes returned in Nak response PayloadHeader.data[0].
	enum ErrorCode : uint8_t {
		kErrNone,
		kErrFail,			///< Unknown failure
		kErrFailErrno,			///< Command failed, errno sent back in PayloadHeader.data[1]
		kErrInvalidDataSize,		///< PayloadHeader.size is invalid
		kErrInvalidSession,		///< Session is not currently open
		kErrNoSessionsAvailable,	///< All available Sessions in use
		kErrEOF,			///< Offset past end of file for List and Read commands
		kErrUnknownCommand,		///< Unknown command opcode
		kErrFailFileExists,		///< File/directory exists already
		kErrFailFileProtected,		///< File/directory is write protected
		kErrFileNotFound                ///< File/directory not found
	};

private:

	void		_process_request(mavlink_file_transfer_protocol_t *ftp_req, uint8_t target_system_id, uint8_t target_comp_id);
	void		_reply(mavlink_file_transfer_protocol_t *ftp_req);

	// ErrorCode	_workList(PayloadHeader *payload);
	ErrorCode	_workOpen(PayloadHeader *payload, int oflag);
	ErrorCode	_workRead(PayloadHeader *payload);
	// ErrorCode	_workBurst(PayloadHeader *payload, uint8_t target_system_id, uint8_t target_component_id);
	// ErrorCode	_workWrite(PayloadHeader *payload);
	// ErrorCode	_workTerminate(PayloadHeader *payload);
	// ErrorCode	_workReset(PayloadHeader *payload);
	// ErrorCode	_workRemoveDirectory(PayloadHeader *payload);
	// ErrorCode	_workCreateDirectory(PayloadHeader *payload);
	// ErrorCode	_workRemoveFile(PayloadHeader *payload);
	// ErrorCode	_workTruncateFile(PayloadHeader *payload);
	// ErrorCode	_workRename(PayloadHeader *payload);
	// ErrorCode	_workCalcFileCRC32(PayloadHeader *payload);

	/// @brief Maximum data size in RequestHeader::data
	static const uint8_t	kMaxDataLength = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(PayloadHeader);

	struct SessionInfo {
		int		fd;
		uint32_t	file_size;
		bool		stream_download;
		uint32_t	stream_offset;
		uint16_t	stream_seq_number;
		uint8_t		stream_target_system_id;
		uint8_t     stream_target_component_id;
		unsigned	stream_chunk_transmitted;
	};
	struct SessionInfo _session_info {};	///< Session info, fd=-1 for no active session

	/* do not allow copying this class */
	MavlinkFTP(const MavlinkFTP &);
	MavlinkFTP operator=(const MavlinkFTP &);

	static constexpr const char _root_dir[] = "";

	bool _last_reply_valid = false;
	uint8_t _last_reply[MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN - MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN
								      + sizeof(PayloadHeader) + sizeof(uint32_t)];

	std::queue<mavlink_file_transfer_protocol_t> m_local_queue;
};