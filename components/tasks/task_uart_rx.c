#include "task_uart_rx.h"
#include "../../main/ecu_common.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "UART_RX";

void task_uart_rx(void *pv)
{
    uint8_t buf[64];
    ESP_LOGI(TAG, "Tâche démarrée (prio 4)");

    while (1) {
        /*
         * Lecture non-bloquante avec timeout court.
         * On utilise uart_read_bytes plutôt qu'une ISR pure
         * pour rester compatible avec le driver ESP-IDF UART.
         * Le driver bufferise en interne, on vide à chaque tour.
         */
        int n = uart_read_bytes(UART_NUM_ECU, buf, sizeof(buf),
                                pdMS_TO_TICKS(10));
        if (n <= 0)
            continue;

        for (int i = 0; i < n; i++) {
            if (xQueueSend(q_bytes, &buf[i], 0) != pdTRUE) {
                // Queue pleine : trame perdue
                ecu_stats.rx_dropped++;
            }
        }
    }
}
