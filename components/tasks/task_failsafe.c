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

    // 1. Forcer mode OFF et output 0
    xSemaphoreTake(mutex_vars, portMAX_DELAY);
    ecu_state.mode   = MODE_OFF;
    ecu_state.output = 0.0f;
    xSemaphoreGive(mutex_vars);

    // 2. Vider la queue output pour que TX n'envoie plus rien
    float dummy;
    while (xQueueReceive(q_output, &dummy, 0) == pdTRUE) {}

    // 3. Envoyer ALERT 0x85 immédiatement via q_tx
    ecu_frame_t alert;
    alert.type = MSG_ALARM;
    alert.len  = (uint8_t)strnlen(reason, MAX_PAYLOAD_LEN);
    memcpy(alert.payload, reason, alert.len);

    // Envoi haute priorité : on pousse en tête de queue si possible
    if (xQueueSendToFront(q_tx, &alert, 0) != pdTRUE) {
        // Queue pleine : on essaie quand même normalement
        xQueueSend(q_tx, &alert, 0);
    }
}

/* ─── ISR GPIO (contexte interruption) ──────────────────────────── */
void IRAM_ATTR failsafe_gpio_isr(void *arg)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(sem_gpio, &woken);
    // Forcer un context switch si Failsafe (prio 5) était en attente
    portYIELD_FROM_ISR(woken);
}

/* ─── Tâche principale ───────────────────────────────────────────── */
void task_failsafe(void *pv)
{
    ESP_LOGI(TAG, "Tâche démarrée (prio 5, watchdog %dms)",
             WATCHDOG_TIMEOUT_MS);

    while (1) {
        /*
         * ulTaskNotifyTake attend une notification du Parser
         * (envoyée à chaque trame valide reçue).
         * Si aucune notification pendant WATCHDOG_TIMEOUT_MS → failsafe.
         */
        uint32_t notified = ulTaskNotifyTake(pdTRUE,
                                pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS));

        if (notified == 0) {
            // Timeout : aucune trame valide depuis 2 secondes
            trigger_failsafe("TIMEOUT: no valid frame for 2s");
            // Après failsafe, on attend un MODE_SET pour reprendre
            // Le Parser continuera à notifier quand le trafic reprend
        }

        // Vérification GPIO en parallèle (non-bloquant)
        if (xSemaphoreTake(sem_gpio, 0) == pdTRUE) {
            trigger_failsafe("GPIO: external error signal");
        }
    }
}
