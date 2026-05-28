#include "task_tx.h"
#include "../../main/ecu_common.h"
#include "../protocol/protocol.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "TX";

/* Envoie une trame brute sur le bus UART de façon atomique */
static void send_raw(const raw_frame_t *frame)
{
    xSemaphoreTake(mutex_uart_tx, portMAX_DELAY);
    uart_write_bytes(UART_NUM_ECU,
                     (const char *)frame->raw,
                     frame->raw_len);
    xSemaphoreGive(mutex_uart_tx);
}

void task_tx(void *pv)
{
    ESP_LOGI(TAG, "Tâche démarrée (prio 4)");

    float       output;
    ecu_frame_t msg;
    raw_frame_t raw;

    while (1) {
        /*
         * On poll les deux queues en alternance avec un timeout court.
         * Priorité à q_output (OUTPUT du PID toutes les 100ms),
         * puis q_tx (STATS toutes les 1s, ALERT immédiate).
         */

        // 1. OUTPUT du PID
        if (xQueueReceive(q_output, &output, pdMS_TO_TICKS(10)) == pdTRUE) {
            protocol_build_float(&raw, MSG_OUTPUT, output);
            send_raw(&raw);
        }

        // 2. STATS ou ALERT depuis Stats/Failsafe
        if (xQueueReceive(q_tx, &msg, pdMS_TO_TICKS(5)) == pdTRUE) {
            // La trame est déjà sérialisée dans msg.payload,
            // on doit la reconstruire via protocol
            protocol_build_frame(&raw, msg.type, msg.payload, msg.len);
            send_raw(&raw);
        }
    }
}
