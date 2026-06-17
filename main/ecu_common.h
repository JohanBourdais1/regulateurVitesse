#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <string.h>

/* ─── Constantes protocole ───────────────────────────────────────── */
#define UART_NUM_ECU        UART_NUM_0
#define UART_TX_PIN         UART_PIN_NO_CHANGE
#define UART_RX_PIN         UART_PIN_NO_CHANGE
#define UART_BAUD_RATE      115200
#define UART_BUF_SIZE       1024

#define FRAME_START         0xAA
#define MSG_SETPOINT        0x01
#define MSG_SPEED           0x02
#define MSG_MODE_SET        0x05
#define MSG_OUTPUT          0x80
#define MSG_STATS           0x83
#define MSG_ALARM           0x85
#define MSG_DBG             0xFF
#define FAILSAFE_GPIO_PIN   4


typedef enum {
    MODE_OFF    = 0,
    MODE_MANUAL = 1,
    MODE_AUTO   = 2,
} ecu_mode_t;


#define MAX_PAYLOAD_LEN 128

typedef struct {
    uint8_t  type;
    uint8_t  payload[MAX_PAYLOAD_LEN];
    uint8_t  len;
} ecu_frame_t;


typedef struct {
    float      setpoint;
    float      speed;
    float      output;
    ecu_mode_t mode;
} ecu_state_t;


typedef struct {
    volatile uint32_t rx_valid;
    volatile uint32_t rx_crc_err;
    volatile uint32_t rx_dropped;
    volatile uint32_t tx_output;
} ecu_stats_t;


extern QueueHandle_t  q_bytes;
extern QueueHandle_t  q_output;
extern QueueHandle_t  q_tx;

extern SemaphoreHandle_t mutex_vars;
extern SemaphoreHandle_t mutex_uart_tx;

/* Bits de notification pour task_failsafe */
#define FAILSAFE_WATCHDOG_BIT  0x01u
#define FAILSAFE_GPIO_BIT      0x02u

extern TaskHandle_t     task_failsafe_handle;
extern TaskHandle_t     task_uart_rx_handle;

extern ecu_state_t      ecu_state;
extern ecu_stats_t      ecu_stats;
