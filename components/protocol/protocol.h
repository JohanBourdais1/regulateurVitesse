#pragma once

#include <stdint.h>
#include <stddef.h>
#include "../../main/ecu_common.h"

/*
 * Format trame :
 * [0xAA][LEN: 2 octets LE][TYPE: 1 octet][PAYLOAD: N octets][CRC: XOR]
 * LEN = taille de (TYPE + PAYLOAD)
 * CRC = XOR de tous les octets sauf le START (0xAA)
 */

/* Buffer brut max pour une trame sérialisée */
#define FRAME_MAX_RAW  (1 + 2 + 1 + MAX_PAYLOAD_LEN + 1)

typedef struct {
    uint8_t raw[FRAME_MAX_RAW];
    uint8_t raw_len;
} raw_frame_t;

/* ─── Construction ───────────────────────────────────────────────── */

/**
 * Construit une trame brute prête à envoyer sur le bus.
 * @param out       trame sérialisée (sortie)
 * @param type      ID du message (ex: MSG_OUTPUT)
 * @param payload   données brutes
 * @param plen      taille du payload en octets
 */
void protocol_build_frame(raw_frame_t *out, uint8_t type,
                           const uint8_t *payload, uint8_t plen);

/**
 * Raccourci : construit une trame avec un float (OUTPUT, SPEED, SETPOINT).
 */
void protocol_build_float(raw_frame_t *out, uint8_t type, float value);

/**
 * Raccourci : construit une trame avec une chaîne (ALARM, DBG).
 */
void protocol_build_string(raw_frame_t *out, uint8_t type, const char *msg);

/**
 * Construit une trame STATS avec 4 compteurs uint32.
 */
void protocol_build_stats(raw_frame_t *out,
                          uint32_t rx_valid, uint32_t rx_crc_err,
                          uint32_t rx_dropped, uint32_t tx_output);

/* ─── CRC ────────────────────────────────────────────────────────── */

/**
 * Calcule le CRC XOR sur les octets fournis (sans le START).
 */
uint8_t protocol_crc(const uint8_t *data, size_t len);
