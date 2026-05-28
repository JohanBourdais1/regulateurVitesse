#include "ecu_common.h"
#include "../protocol/protocol.h"
#include "../tasks/task_uart_rx.h"
#include "../tasks/task_parser.h"
#include "../tasks/task_pid.h"
#include "../tasks/task_tx.h"
#include "../tasks/task_failsafe.h"
#include "../tasks/task_stats.h"

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "ECU_MAIN";

/* ─── Définitions des handles globaux ───────────────────────────── */
QueueHandle_t     q_bytes;
QueueHandle_t     q_output;
QueueHandle_t     q_tx;
SemaphoreHandle_t mutex_vars;
SemaphoreHandle_t mutex_uart_tx;
SemaphoreHandle_t sem_gpio;
TaskHandle_t      task_failsafe_handle;
TaskHandle_t      task_uart_rx_handle;
ecu_state_t       ecu_state;
ecu_stats_t       ecu_stats;

/* ─── Configuration UART ─────────────────────────────────────────── */
static void uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_driver_install(UART_NUM_ECU, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_ECU, &cfg);
    uart_set_pin(UART_NUM_ECU, UART_TX_PIN, UART_RX_PIN,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "UART initialisé à %d bauds", UART_BAUD_RATE);
}

/* ─── Configuration GPIO Failsafe ────────────────────────────────── */
static void gpio_failsafe_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << FAILSAFE_GPIO_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(FAILSAFE_GPIO_PIN, failsafe_gpio_isr, NULL);
    ESP_LOGI(TAG, "GPIO failsafe configuré sur pin %d", FAILSAFE_GPIO_PIN);
}

/* ─── Création des primitives IPC ────────────────────────────────── */
static void ipc_init(void)
{
    q_bytes       = xQueueCreate(256, sizeof(uint8_t));
    q_output      = xQueueCreate(5,   sizeof(float));
    q_tx          = xQueueCreate(10,  sizeof(ecu_frame_t));
    mutex_vars    = xSemaphoreCreateMutex();
    mutex_uart_tx = xSemaphoreCreateMutex();
    sem_gpio      = xSemaphoreCreateBinary();

    configASSERT(q_bytes);
    configASSERT(q_output);
    configASSERT(q_tx);
    configASSERT(mutex_vars);
    configASSERT(mutex_uart_tx);
    configASSERT(sem_gpio);

    ESP_LOGI(TAG, "Primitives IPC créées");
}

/* ─── Initialisation état ECU ────────────────────────────────────── */
static void state_init(void)
{
    ecu_state.setpoint = 0.0f;
    ecu_state.speed    = 0.0f;
    ecu_state.output   = 0.0f;
    ecu_state.mode     = MODE_OFF;
    memset(&ecu_stats, 0, sizeof(ecu_stats));
}

/* ─── Création des tâches ────────────────────────────────────────── */
static void tasks_init(void)
{
    // Prio 5 — temps réel critique
    xTaskCreate(task_pid,      "PID",      2048, NULL, 5, NULL);
    xTaskCreate(task_failsafe, "FAILSAFE", 3072, NULL, 5, &task_failsafe_handle);

    // Prio 4 — communication
    xTaskCreate(task_uart_rx,  "UART_RX",  2048, NULL, 4, &task_uart_rx_handle);
    xTaskCreate(task_tx,       "TX",       2048, NULL, 4, NULL);

    // Prio 3 — traitement
    xTaskCreate(task_parser,   "PARSER",   3072, NULL, 3, NULL);

    // Prio 1 — monitoring
    xTaskCreate(task_stats,    "STATS",    2048, NULL, 1, NULL);

    ESP_LOGI(TAG, "6 tâches démarrées");
}

void app_main(void)
{
    ESP_LOGI(TAG, "=== ECU Régulateur de Vitesse — EPITA 2026 ===");

    state_init();
    ipc_init();
    uart_init();
    gpio_failsafe_init();
    tasks_init();

    // app_main se termine — le scheduler FreeRTOS prend la main
}
