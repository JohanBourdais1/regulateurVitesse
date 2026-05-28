#pragma once
#include "freertos/FreeRTOS.h"

void task_failsafe(void *pv);

/* ISR GPIO — doit être appelée depuis gpio_isr_handler_add() */
void failsafe_gpio_isr(void *arg);
