#pragma once

/* Firmware Update file format:
 *
 *    4 byte header: 'D', 'A', 'I', 0x00
 *    32-bit format version
 *    entry[0]: 32-bit entry type, 32-bit entry size, entry payload
 *    entry[1]
 *    ...
 *    entry[n]
 *    TODO: 16 bytes MD5 hash for integrity check
 */

#define FW_FILE_HEADER    {'D', 'A', 'I', 0x00}
#define FW_FORMAT_VERSION 0 // 32-bit

#define FW_ENTRY_START_OFFSET 8 // after header and version

enum fw_entry_type {
    FW_DEVICE_APP = 1,
    FW_CONFIG,
    FW_NN_BLOB,
};

struct fw_entry_header {
    uint32_t type; // enum fw_entry_type
    uint32_t size;
    uint8_t  payload[];
};
