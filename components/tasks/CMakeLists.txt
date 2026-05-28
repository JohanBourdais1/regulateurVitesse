idf_component_register(
    SRCS
        "task_uart_rx.c"
        "task_parser.c"
        "task_pid.c"
        "task_tx.c"
        "task_failsafe.c"
        "task_stats.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        freertos
        protocol
        pid
        esp_driver_uart
        esp_driver_gpio
)