# ECU Régulateur de Vitesse — EPITA 2026

Projet ESP32 / ESP-IDF / FreeRTOS — TP Conception d'un ECU Temps Réel.

## Structure du projet

```
ecu_project/
├── CMakeLists.txt
├── main/
│   ├── CMakeLists.txt
│   ├── ecu_common.h      ← types, handles globaux, constantes
│   └── main.c            ← init + lancement des 6 tâches
└── components/
    ├── protocol/
    │   ├── protocol.h    ← construction/CRC des trames
    │   └── protocol.c
    ├── pid/
    │   ├── pid.h         ← algorithme PID avec anti-windup
    │   └── pid.c
    └── tasks/
        ├── task_uart_rx  ← prio 4 : lecture UART
        ├── task_parser   ← prio 3 : décodage trames (machine à états)
        ├── task_pid      ← prio 5 : régulation 100ms strict
        ├── task_tx       ← prio 4 : émission bus (mutex)
        ├── task_failsafe ← prio 5 : watchdog 2s + GPIO ISR
        └── task_stats    ← prio 1 : télémétrie 1s
```

## Compilation et flash

```bash
# 1. Activer l'environnement ESP-IDF
cd ~/esp/esp-idf && . ./export.sh

# 2. Aller dans le projet
cd ecu_project

# 3. Configurer la cible ESP32
idf.py set-target esp32

# 4. (Optionnel) Modifier les pins UART dans ecu_common.h
#    UART_TX_PIN = 17, UART_RX_PIN = 16 par défaut

# 5. Compiler
idf.py build

# 6. Flasher et monitorer
idf.py -p /dev/ttyUSB0 flash monitor
```

## Broches matérielles

| Signal | Pin ESP32 |
|---|---|
| UART TX | GPIO 17 |
| UART RX | GPIO 16 |
| Failsafe GPIO | GPIO 4 |

Modifier `UART_TX_PIN`, `UART_RX_PIN`, `FAILSAFE_GPIO_PIN` dans `main/ecu_common.h`.

## Test avec le script Python

```bash
pip install pyserial
python3 test_ecu.py
```

Le script injecte des trames normales, fragmentées, CRC erronés, et un flood.

## Paramètres PID

Dans `components/tasks/task_pid.c` :

```c
#define PID_KP  1.0f
#define PID_KI  0.1f
#define PID_KD  0.01f
```

## Architecture FreeRTOS

| Tâche | Prio | Période | Mécanisme |
|---|---|---|---|
| PID | 5 | 100 ms strict | vTaskDelayUntil |
| Failsafe | 5 | Événementielle | ulTaskNotifyTake + sem GPIO |
| UART RX | 4 | Asynchrone | uart_read_bytes |
| TX | 4 | Sur événement | xQueueReceive + mutex |
| Parser | 3 | Sur événement | xQueueReceive (machine à états) |
| Stats | 1 | 1 s | vTaskDelay |
