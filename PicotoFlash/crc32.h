#ifndef MINI_CRC32_H
#define MINI_CRC32_H

#include <stdint.h>
#include <stddef.h>

static inline uint32_t crc32_init(void) { return 0xFFFFFFFFu; }

static inline uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        uint32_t c = (crc ^ data[i]) & 0xFFu;
        for (int k = 0; k < 8; k++) {
            c = (c >> 1) ^ (0xEDB88320u & (-(int)(c & 1u)));
        }
        crc = (crc >> 8) ^ c;
    }
    return crc;
}

static inline uint32_t crc32_final(uint32_t crc) { return crc ^ 0xFFFFFFFFu; }

#endif /* MINI_CRC32_H */
