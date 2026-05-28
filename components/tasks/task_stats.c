#include "task_stats.h"
#include "../../main/ecu_common.h"
#include "../protocol/protocol.h"
#include "esp_log.h"

static const char *TAG = "STATS";

void task_stats(void *pv)
{
    ESP_LOGI(TAG, "Tâche démarrée (prio 1, période 1s)");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Lecture snapshot des compteurs
        uint32_t rx_valid   = ecu_stats.rx_valid;
        uint32_t rx_crc_err = ecu_stats.rx_crc_err;
        uint32_t rx_dropped = ecu_stats.rx_dropped;
        uint32_t tx_output  = ecu_stats.tx_output;

        ESP_LOGI(TAG, "rx_ok=%lu crc_err=%lu dropped=%lu tx_out=%lu",
                 rx_valid, rx_crc_err, rx_dropped, tx_output);

        // Construction de la trame STATS 0x83 (4 x uint32 LE)
        ecu_frame_t msg;
        msg.type = MSG_STATS;
        msg.len  = 16;
        uint32_t vals[4] = {rx_valid, rx_crc_err, rx_dropped, tx_output};
        memcpy(msg.payload, vals, 16);

        // Envoi non-bloquant — si queue TX pleine, on skip cette seconde
        if (xQueueSend(q_tx, &msg, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Queue TX pleine, STATS droppées");
        }
    }
}
