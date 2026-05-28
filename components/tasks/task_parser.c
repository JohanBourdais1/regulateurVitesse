#include "task_parser.h"
#include "../../main/ecu_common.h"
#include "../protocol/protocol.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "PARSER";

/* ─── Machine à états ────────────────────────────────────────────── */
typedef enum {
    STATE_WAIT_START = 0,
    STATE_READ_LEN_LO,
    STATE_READ_LEN_HI,
    STATE_READ_TYPE,
    STATE_READ_PAYLOAD,
    STATE_CHECK_CRC,
} parser_state_t;

/* ─── Traitement d'une trame valide ──────────────────────────────── */
static void process_frame(uint8_t type, const uint8_t *payload, uint8_t plen)
{
    switch (type) {

    case MSG_SETPOINT:
        if (plen == 4) {
            float val;
            memcpy(&val, payload, 4);
            xSemaphoreTake(mutex_vars, portMAX_DELAY);
            ecu_state.setpoint = val;
            xSemaphoreGive(mutex_vars);
            ESP_LOGD(TAG, "SETPOINT = %.2f", val);
        }
        break;

    case MSG_SPEED:
        if (plen == 4) {
            float val;
            memcpy(&val, payload, 4);
            xSemaphoreTake(mutex_vars, portMAX_DELAY);
            ecu_state.speed = val;
            xSemaphoreGive(mutex_vars);
        }
        break;

    case MSG_MODE_SET:
        if (plen == 1) {
            ecu_mode_t new_mode = (ecu_mode_t)payload[0];
            xSemaphoreTake(mutex_vars, portMAX_DELAY);
            ecu_mode_t old_mode = ecu_state.mode;
            ecu_state.mode = new_mode;
            // Transition hors AUTO : couper la commande moteur
            if (new_mode != MODE_AUTO)
                ecu_state.output = 0.0f;
            xSemaphoreGive(mutex_vars);
            ESP_LOGI(TAG, "Mode: %d → %d", old_mode, new_mode);
        }
        break;

    default:
        // ID inconnu : rejet silencieux, déjà compté dans rx_crc_err
        break;
    }
}

/* ─── Tâche principale ───────────────────────────────────────────── */
void task_parser(void *pv)
{
    ESP_LOGI(TAG, "Tâche démarrée (prio 3)");

    parser_state_t state = STATE_WAIT_START;
    uint8_t  byte;
    uint8_t  buf[MAX_PAYLOAD_LEN + 4]; // LEN(2) + TYPE(1) + PAYLOAD + CRC(1)
    uint16_t expected_len  = 0;  // LEN lu dans la trame
    uint8_t  payload_type  = 0;
    uint8_t  payload_buf[MAX_PAYLOAD_LEN];
    uint16_t payload_idx   = 0;
    uint16_t buf_idx       = 0;  // index pour recalcul CRC

    while (1) {
        if (xQueueReceive(q_bytes, &byte, portMAX_DELAY) != pdTRUE)
            continue;

        switch (state) {

        case STATE_WAIT_START:
            if (byte == FRAME_START) {
                buf_idx = 0;
                state   = STATE_READ_LEN_LO;
            }
            break;

        case STATE_READ_LEN_LO:
            expected_len = byte;
            buf[buf_idx++] = byte;
            state = STATE_READ_LEN_HI;
            break;

        case STATE_READ_LEN_HI:
            expected_len |= ((uint16_t)byte << 8);
            buf[buf_idx++] = byte;
            // Sanity check : LEN doit être entre 1 et MAX_PAYLOAD_LEN+1
            if (expected_len == 0 || expected_len > (MAX_PAYLOAD_LEN + 1)) {
                ESP_LOGW(TAG, "LEN invalide: %d", expected_len);
                ecu_stats.rx_crc_err++;
                state = STATE_WAIT_START;
            } else {
                state = STATE_READ_TYPE;
            }
            break;

        case STATE_READ_TYPE:
            payload_type   = byte;
            buf[buf_idx++] = byte;
            payload_idx    = 0;
            // Si LEN == 1, il n'y a pas de payload
            if (expected_len == 1)
                state = STATE_CHECK_CRC;
            else
                state = STATE_READ_PAYLOAD;
            break;

        case STATE_READ_PAYLOAD:
            payload_buf[payload_idx++] = byte;
            buf[buf_idx++]             = byte;
            // -1 car LEN inclut le TYPE
            if (payload_idx >= (expected_len - 1))
                state = STATE_CHECK_CRC;
            break;

        case STATE_CHECK_CRC: {
            // buf contient : LEN_LO, LEN_HI, TYPE, PAYLOAD...
            // CRC = XOR de buf[0..buf_idx-1]
            uint8_t computed = protocol_crc(buf, buf_idx);
            if (computed == byte) {
                // Trame valide
                ecu_stats.rx_valid++;
                process_frame(payload_type, payload_buf,
                              (uint8_t)(expected_len - 1));
                // Reset watchdog Failsafe
                xTaskNotify(task_failsafe_handle, 0, eNoAction);
            } else {
                ecu_stats.rx_crc_err++;
                ESP_LOGW(TAG, "CRC err: got 0x%02X expected 0x%02X",
                         byte, computed);
            }
            state = STATE_WAIT_START;
            break;
        }

        default:
            state = STATE_WAIT_START;
            break;
        }
    }
}
