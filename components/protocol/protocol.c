#include "protocol.h"
#include <string.h>

uint8_t protocol_crc(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; i++)
        crc ^= data[i];
    return crc;
}

void protocol_build_frame(raw_frame_t *out, uint8_t type,
                           const uint8_t *payload, uint8_t plen)
{
    /*
     * Structure :
     * [START=0xAA] [LEN_LO] [LEN_HI] [TYPE] [PAYLOAD...] [CRC]
     * LEN = 1 (TYPE) + plen (PAYLOAD)
     */
    uint16_t len = (uint16_t)(1 + plen);

    out->raw[0] = FRAME_START;
    out->raw[1] = (uint8_t)(len & 0xFF);
    out->raw[2] = (uint8_t)((len >> 8) & 0xFF);
    out->raw[3] = type;
    if (plen > 0 && payload != NULL)
        memcpy(&out->raw[4], payload, plen);

    // CRC sur tout sauf le START
    out->raw[4 + plen] = protocol_crc(&out->raw[1], 3 + plen);
    out->raw_len = (uint8_t)(5 + plen);
}

void protocol_build_float(raw_frame_t *out, uint8_t type, float value)
{
    uint8_t buf[4];
    memcpy(buf, &value, 4); // IEEE754 little endian
    protocol_build_frame(out, type, buf, 4);
}

void protocol_build_string(raw_frame_t *out, uint8_t type, const char *msg)
{
    uint8_t plen = (uint8_t)strnlen(msg, MAX_PAYLOAD_LEN);
    protocol_build_frame(out, type, (const uint8_t *)msg, plen);
}

void protocol_build_stats(raw_frame_t *out,uint32_t rx_valid, uint32_t rx_crc_err,
                          uint32_t rx_dropped, uint32_t tx_output)
{
    uint8_t buf[16];
    memcpy(buf +  0, &rx_valid,   4);
    memcpy(buf +  4, &rx_crc_err, 4);
    memcpy(buf +  8, &rx_dropped, 4);
    memcpy(buf + 12, &tx_output,  4);
    protocol_build_frame(out, MSG_STATS, buf, 16);
}
