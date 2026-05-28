#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <stdint.h>
#include <string.h>

/* ─── Constantes protocole ───────────────────────────────────────── */
#define UART_NUM_ECU        UART_NUM_1
#define UART_TX_PIN         17
#define UART_RX_PIN         16
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

/* ─── Modes ECU ──────────────────────────────────────────────────── */
typedef enum {
    MODE_OFF    = 0,
    MODE_MANUAL = 1,
    MODE_AUTO   = 2,
} ecu_mode_t;

/* ─── Trame interne ──────────────────────────────────────────────── */
#define MAX_PAYLOAD_LEN 128

typedef struct {
    uint8_t  type;
    uint8_t  payload[MAX_PAYLOAD_LEN];
    uint8_t  len;
} ecu_frame_t;

/* ─── Variables partagées (protégées par mutex_vars) ─────────────── */
typedef struct {
    float      setpoint;
    float      speed;
    float      output;
    ecu_mode_t mode;
} ecu_state_t;

/* ─── Compteurs télémétrie (accès atomique uint32) ───────────────── */
typedef struct {
    volatile uint32_t rx_valid;
    volatile uint32_t rx_crc_err;
    volatile uint32_t rx_dropped;
    volatile uint32_t tx_output;
} ecu_stats_t;

/* ─── Handles globaux ────────────────────────────────────────────── */
extern QueueHandle_t    q_bytes;       // uint8_t  — UART RX → Parser
extern QueueHandle_t    q_output;      // float    — PID → TX
extern QueueHandle_t    q_tx;          // ecu_frame_t — Stats/Failsafe → TX

extern SemaphoreHandle_t mutex_vars;   // protège ecu_state
extern SemaphoreHandle_t mutex_uart_tx;// accès exclusif bus UART TX
extern SemaphoreHandle_t sem_gpio;     // ISR GPIO → Failsafe

extern TaskHandle_t     task_failsafe_handle;
extern TaskHandle_t     task_uart_rx_handle;

extern ecu_state_t      ecu_state;
extern ecu_stats_t      ecu_stats;
