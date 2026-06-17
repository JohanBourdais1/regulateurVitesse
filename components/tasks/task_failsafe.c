#include "task_failsafe.h"
#include "../../main/ecu_common.h"
#include "../protocol/protocol.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "FAILSAFE";

#define WATCHDOG_TIMEOUT_MS  2000

/* ─── Déclenchement failsafe ─────────────────────────────────────── */
static void trigger_failsafe(const char *reason)
{
    ESP_LOGE(TAG, "FAILSAFE déclenché : %s", reason);

    xSemaphoreTake(mutex_vars, portMAX_DELAY);
    ecu_state.mode   = MODE_OFF;
    ecu_state.output = 0.0f;
    xSemaphoreGive(mutex_vars);

    float dummy;
    while (xQueueReceive(q_output, &dummy, 0) == pdTRUE) {}
    float zero = 0.0f;
    xQueueSend(q_output, &zero, 0);

    ecu_frame_t alert;
    alert.type = MSG_ALARM;
    alert.len  = (uint8_t)strnlen(reason, MAX_PAYLOAD_LEN);
    memcpy(alert.payload, reason, alert.len);

    if (xQueueSendToFront(q_tx, &alert, 0) != pdTRUE) {
        xQueueSend(q_tx, &alert, 0);
    }
}

/* ─── ISR GPIO (contexte interruption) ──────────────────────────── */
/*
 * On notifie directement task_failsafe via FAILSAFE_GPIO_BIT.
 * portYIELD_FROM_ISR force un context switch immédiat vers la tâche
 * (prio 5) dès la fin de l'ISR, garantissant une réponse < 5 ms.
 */
void IRAM_ATTR failsafe_gpio_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    xTaskNotifyFromISR(task_failsafe_handle,
                       FAILSAFE_GPIO_BIT,
                       eSetBits,
                       &woken);
    portYIELD_FROM_ISR(woken);
}

/*Tâche principale*/
void task_failsafe(void *pv)
{
    ESP_LOGI(TAG, "Tâche démarrée (prio 5, watchdog %dms)",
             WATCHDOG_TIMEOUT_MS);

    while (1) {
        /*
         * xTaskNotifyWait attend :
         *   - FAILSAFE_WATCHDOG_BIT (0x01) : envoyé par le Parser à chaque
         *     trame valide → reset du watchdog 2 s.
         *   - FAILSAFE_GPIO_BIT    (0x02) : envoyé par l'ISR GPIO
         *     directement → réponse garantie < 5 ms.
         *
         * Timeout WATCHDOG_TIMEOUT_MS sans notification → failsafe.
         * Tous les bits sont effacés à la sortie (ulBitsToClearOnExit).
         */
        uint32_t bits = 0;
        BaseType_t notified = xTaskNotifyWait(0,
                                              0xFFFFFFFFu,
                                              &bits,
                                              pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS));

        if (notified == pdFALSE) {
            // Timeout : aucune trame valide depuis 2 secondes
            trigger_failsafe("TIMEOUT: no valid frame for 2s");
        }

        if (bits & FAILSAFE_GPIO_BIT) {
            // Signal GPIO reçu directement depuis l'ISR
            trigger_failsafe("GPIO: external error signal");
        }

        // FAILSAFE_WATCHDOG_BIT : trame valide reçue → watchdog reset implicite
        // (xTaskNotifyWait repart avec un nouveau délai de 2 s)
    }
}
