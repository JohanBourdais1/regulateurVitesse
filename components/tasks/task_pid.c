#include "task_pid.h"
#include "../../main/ecu_common.h"
#include "../pid/pid.h"
#include "esp_log.h"

static const char *TAG = "PID";

#define PID_PERIOD_MS   100
#define PID_KP          1.0f
#define PID_KI          0.1f
#define PID_KD          0.01f
#define PID_DT          (PID_PERIOD_MS / 1000.0f)
#define PID_OUT_MIN     0.0f
#define PID_OUT_MAX     255.0f

void task_pid(void *pv)
{
    ESP_LOGI(TAG, "Tâche démarrée (prio 5, période %dms)", PID_PERIOD_MS);

    pid_t pid;
    pid_init(&pid, PID_KP, PID_KI, PID_KD,
             PID_DT, PID_OUT_MIN, PID_OUT_MAX);

    TickType_t  last_wake  = xTaskGetTickCount();
    ecu_mode_t  prev_mode  = MODE_OFF;

    while (1) {
        // Attente strictement périodique — garantit le jitter < 5ms
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(PID_PERIOD_MS));

        // Lecture non-bloquante des vars partagées
        // Timeout 0 : si le mutex est pris, on utilise les anciennes valeurs
        // plutôt que de rater notre fenêtre temps réel.
        float sp   = 0.0f;
        float spd  = 0.0f;
        ecu_mode_t mode_now = MODE_OFF;

        if (xSemaphoreTake(mutex_vars, 0) == pdTRUE) {
            sp       = ecu_state.setpoint;
            spd      = ecu_state.speed;
            mode_now = ecu_state.mode;
            xSemaphoreGive(mutex_vars);
        } else {
            // Mutex occupé : on skip cette itération
            continue;
        }

        // Réinitialisation PID lors d'une reprise en mode AUTO
        if (mode_now == MODE_AUTO && prev_mode != MODE_AUTO) {
            pid_reset(&pid);
            ESP_LOGI(TAG, "PID réinitialisé (reprise AUTO)");
        }
        prev_mode = mode_now;

        // Hors mode AUTO : on ne calcule rien
        if (mode_now != MODE_AUTO)
            continue;

        // Calcul PID
        float output = pid_compute(&pid, sp, spd);

        // Mise à jour de l'output dans l'état partagé
        if (xSemaphoreTake(mutex_vars, 0) == pdTRUE) {
            ecu_state.output = output;
            xSemaphoreGive(mutex_vars);
        }

        // Envoi dans la queue TX (non-bloquant)
        if (xQueueSend(q_output, &output, 0) != pdTRUE) {
            ESP_LOGW(TAG, "Queue output pleine, output droppé");
        } else {
            ecu_stats.tx_output++;
        }

        ESP_LOGD(TAG, "sp=%.1f spd=%.1f out=%.1f", sp, spd, output);
    }
}
