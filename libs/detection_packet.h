#ifndef DETECTION_PACKET_H_
#define DETECTION_PACKET_H_

// ============================================================================
// Detection stream over MAVLINK_MSG_ID_V2_EXTENSION (248)
//
// !!! WIRE FORMAT — must match payloadsdk_enum.h in the payload app byte for byte.
// !!! Changing any field: bump DET_PACKET_VERSION and update both sides.
// ============================================================================
#include <stdint.h>

#define DET_PACKET_MAGIC   0x4744  // "GD"
#define DET_PACKET_VERSION 1
#define DET_MAX_BOXES      20
#define DET_MSG_TYPE       0x4001  // V2_EXTENSION message_type used for detections

#pragma pack(push, 1)
typedef struct {
    uint16_t x;             // pixels, top-left corner (see DET_REF_WIDTH/HEIGHT)
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint16_t track_id;      // 0 = no track assigned yet
    uint8_t  class_id;
    uint8_t  confidence;    // 0-255 maps to 0.0-1.0
} det_box_t;                // 12 bytes

typedef struct {
    uint16_t  magic;        // DET_PACKET_MAGIC
    uint8_t   version;      // DET_PACKET_VERSION
    uint8_t   num_boxes;    // number of valid entries in boxes[]
    uint32_t  frame_id;     // increments per packet, lets the client spot drops
    det_box_t boxes[DET_MAX_BOXES];  // 240 bytes
} det_packet_t;             // 248 bytes <= 249 (V2_EXTENSION payload)
#pragma pack(pop)

// Useful size when only n boxes are filled in (header struct = 8 bytes)
#define DET_PACKET_SIZE(n)  (8u + (n) * sizeof(det_box_t))

// Coordinate space the payload reports boxes in. Scale to the actual video
// resolution before drawing.
#define DET_REF_WIDTH   1920
#define DET_REF_HEIGHT  1080

#endif