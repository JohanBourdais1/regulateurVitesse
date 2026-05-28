# Architecture FreeRTOS — ECU Régulateur de Vitesse

## Vue d'ensemble

L'ECU est décomposé en **6 tâches FreeRTOS indépendantes** qui communiquent via des mécanismes de synchronisation (queues, mutex, semaphores). Cette architecture évite toute architecture monolithique et garantit le déterminisme temps réel.

---

## Tableau des tâches

| Tâche | Priorité | Déclenchement | Stack | Mécanisme |
|---|---|---|---|---|
| UART RX | 4 | Asynchrone (ISR) | 2048 B | Queue octets |
| Parser | 3 | Bloquée sur queue | 3072 B | Queue + mutex |
| PID | 5 (max) | 100 ms strict | 2048 B | vTaskDelayUntil |
| TX | 4 | Bloquée sur queue | 2048 B | Mutex UART TX |
| Failsafe | 5 (max) | Événementielle | 1024 B | Semaphore GPIO |
| Stats | 1 (min) | 1 s périodique | 2048 B | vTaskDelay |

---

## Mécanismes IPC

| Mécanisme | Type | Entre | Contenu |
|---|---|---|---|
| `q_bytes` | Queue 256 items | UART RX → Parser | `uint8_t` (octets bruts) |
| `q_output` | Queue 5 items | PID → TX | `float` (commande moteur) |
| `q_tx` | Queue 10 items | Stats/Failsafe → TX | `ecu_frame_t` (trame complète) |
| `mutex_vars` | Mutex | Parser ↔ PID | Accès à setpoint, speed, mode |
| `mutex_uart_tx` | Mutex | TX exclusif | Écriture bus UART |
| `sem_gpio` | Semaphore binaire | ISR GPIO → Failsafe | Signal déclenchement |

---

## Détail des tâches

---

### 1. Tâche UART RX

**Priorité :** 4  
**Stack :** 2048 octets  
**Déclenchement :** Asynchrone, réveillée par l'ISR hardware

#### Rôle

Elle lit les octets qui arrivent sur le port série un par un et les pousse dans une queue. C'est la première porte d'entrée de toutes les données reçues. Elle ne fait aucun traitement — juste lire et transmettre.

#### Entrées / Sorties

- **Entrée :** Octets bruts depuis l'UART (ISR hardware)
- **Sortie :** Queue `q_bytes` → Tâche Parser

#### Appels FreeRTOS

| Appel | Pourquoi |
|---|---|
| `uart_driver_install()` | Configure le driver UART au démarrage |
| `uart_read_bytes()` | Lit les octets disponibles dans le buffer UART |
| `xQueueSend(q_bytes, &byte, 0)` | Envoie chaque octet dans la queue. Le `0` = pas d'attente si pleine (on drop et on incrémente le compteur) |
| `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)` | L'ISR réveille la tâche dès qu'un octet arrive |

#### Squelette de code

```c
void task_uart_rx(void *pv) {
    uint8_t byte;
    while (1) {
        // Attend une notification de l'ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Vide le buffer UART
        int n = uart_read_bytes(UART_NUM, &byte, 1, 0);
        if (n > 0) {
            if (xQueueSend(q_bytes, &byte, 0) != pdTRUE)
                stats.rx_dropped++;
        }
    }
}
```

#### Liée à

- **Parser** (via `q_bytes`)

---

### 2. Tâche Parser

**Priorité :** 3  
**Stack :** 3072 octets (plus grande car manipulation de buffers)  
**Déclenchement :** Bloquée sur queue — se réveille dès qu'un octet arrive

#### Rôle

Elle reconstitue les trames octet par octet grâce à une machine à états, vérifie le CRC, identifie le type de message, puis met à jour les variables partagées (`setpoint`, `speed`, `mode`). C'est le décodeur du protocole. Toute trame avec un CRC invalide, une longueur incohérente ou un ID inconnu est rejetée silencieusement.

#### Machine à états interne

```
WAIT_START → READ_LEN → READ_TYPE → READ_PAYLOAD → CHECK_CRC → WAIT_START
```

#### Entrées / Sorties

- **Entrée :** Queue `q_bytes` ← Tâche UART RX
- **Sorties :**
  - `mutex_vars` : mise à jour de `setpoint`, `speed`, `mode`
  - Notification watchdog → Tâche Failsafe
  - Compteurs `stats.rx_valid`, `stats.rx_crc_err`

#### Appels FreeRTOS

| Appel | Pourquoi |
|---|---|
| `xQueueReceive(q_bytes, &byte, portMAX_DELAY)` | Attend le prochain octet. Bloque la tâche si queue vide |
| `xSemaphoreTake(mutex_vars, portMAX_DELAY)` | Verrouille l'accès aux variables partagées avant écriture |
| `xSemaphoreGive(mutex_vars)` | Relâche le verrou après mise à jour |
| `xTaskNotify(failsafe_handle, 0, eNoAction)` | Signale au Failsafe qu'une trame valide vient d'arriver (reset watchdog) |

#### Squelette de code

```c
void task_parser(void *pv) {
    uint8_t byte, buf[128];
    int state = WAIT_START, idx = 0;
    while (1) {
        xQueueReceive(q_bytes, &byte, portMAX_DELAY);
        switch (state) {
            case WAIT_START:
                if (byte == 0xAA) { state = READ_LEN; idx = 0; }
                break;
            case READ_LEN:   /* lire 2 octets LEN */ break;
            case READ_TYPE:  /* lire TYPE */         break;
            case READ_PAYLOAD: /* lire N octets */   break;
            case CHECK_CRC:
                if (crc_ok(buf, idx)) {
                    process_frame(buf); // met à jour les vars partagées
                    xTaskNotify(failsafe_handle, 0, eNoAction);
                    stats.rx_valid++;
                } else {
                    stats.rx_crc_err++;
                }
                state = WAIT_START;
                break;
        }
    }
}
```

#### Liée à

- **UART RX** (reçoit les octets)
- **PID** (écrit `setpoint` et `speed`)
- **Failsafe** (reset watchdog)

---

### 3. Tâche PID

**Priorité :** 5 — priorité maximale  
**Stack :** 2048 octets  
**Déclenchement :** Strictement toutes les 100 ms via `vTaskDelayUntil`

#### Rôle

C'est le cœur du système. Elle lit la consigne et la vitesse, calcule la commande moteur via l'algorithme PID discret, la sature entre 0 et 255, et l'envoie dans la queue TX. Elle ne doit jamais être préemptée pendant son calcul.

#### Algorithme PID discret

```
error     = setpoint - speed
integral += error * dt               (dt = 0.1s)
derivee   = (error - prev_error) / dt
output    = Kp*error + Ki*integral + Kd*derivee
output    = clamp(output, 0, 255)    (saturation)
```

Coefficients par défaut : `Kp = 1.0`, `Ki = 0.1`, `Kd = 0.01`

#### Entrées / Sorties

- **Entrée :** `mutex_vars` (lecture de `setpoint`, `speed`, `mode`)
- **Sortie :** Queue `q_output` → Tâche TX (`float`)

#### Appels FreeRTOS

| Appel | Pourquoi |
|---|---|
| `vTaskDelayUntil(&last, pdMS_TO_TICKS(100))` | Garantit exactement 100 ms entre chaque calcul, même si la tâche a mis du temps à s'exécuter |
| `xSemaphoreTake(mutex_vars, 0)` | Lecture rapide des vars partagées. Le `0` = pas d'attente pour ne pas rater la période de 100 ms |
| `xSemaphoreGive(mutex_vars)` | Relâche le verrou immédiatement après lecture |
| `xQueueSend(q_output, &output, 0)` | Envoie le résultat PID. `0` = non-bloquant, la régulation ne peut pas attendre |

> **Point important :** Le timeout `0` sur `xSemaphoreTake(mutex_vars, 0)` est un choix de conception délibéré. Si le Parser est en train d'écrire au même moment, le PID utilise les anciennes valeurs plutôt que de bloquer et rater sa fenêtre de 100 ms. C'est acceptable car les valeurs changent lentement (vitesse physique d'un véhicule).

#### Squelette de code

```c
void task_pid(void *pv) {
    TickType_t last = xTaskGetTickCount();
    float integral = 0.0f, prev_err = 0.0f;
    const float Kp = 1.0f, Ki = 0.1f, Kd = 0.01f;
    const float dt = 0.1f;

    while (1) {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));

        if (mode != AUTO) continue;

        // Lecture des vars partagées
        xSemaphoreTake(mutex_vars, 0);
        float sp  = setpoint;
        float spd = speed;
        xSemaphoreGive(mutex_vars);

        // Calcul PID
        float err   = sp - spd;
        integral   += err * dt;
        float der   = (err - prev_err) / dt;
        float output = Kp*err + Ki*integral + Kd*der;

        // Saturation + anti-windup
        if (output > 255.0f) { output = 255.0f; if (err > 0) integral -= err * dt; }
        if (output < 0.0f)   { output = 0.0f;   if (err < 0) integral -= err * dt; }

        prev_err = err;
        xQueueSend(q_output, &output, 0);
        stats.tx_output++;
    }
}
```

#### Liée à

- **Parser** (lit `setpoint` et `speed`)
- **TX** (envoie `output`)

---

### 4. Tâche TX

**Priorité :** 4  
**Stack :** 2048 octets  
**Déclenchement :** Bloquée sur queue — envoie dès qu'un message est disponible

#### Rôle

Elle est l'unique entité qui écrit sur le bus UART. Elle reçoit des trames à envoyer depuis plusieurs sources (PID, Stats, Failsafe), les sérialise au format du protocole, et garantit qu'elles ne se mélangent jamais grâce au mutex. Sans ce mutex, deux tâches pourraient écrire en même temps et produire des trames corrompues.

#### Format de trame émise

```
[0xAA] [LEN: 2 octets LE] [TYPE: 1 octet] [PAYLOAD: N octets] [CRC: XOR]
```

#### Entrées / Sorties

- **Entrées :**
  - Queue `q_output` ← PID (`float`)
  - Queue `q_tx` ← Stats et Failsafe (`ecu_frame_t`)
- **Sortie :** Bus UART physique

#### Appels FreeRTOS

| Appel | Pourquoi |
|---|---|
| `xQueueReceive(q_output, &val, portMAX_DELAY)` | Attend le prochain OUTPUT ou message à envoyer |
| `xSemaphoreTake(mutex_uart_tx, portMAX_DELAY)` | Verrou exclusif sur l'UART — une seule trame à la fois, jamais d'entrelacement |
| `uart_write_bytes(UART_NUM, frame, len)` | Envoie la trame complète sur le bus |
| `xSemaphoreGive(mutex_uart_tx)` | Libère le bus pour la prochaine trame |

#### Squelette de code

```c
void task_tx(void *pv) {
    float output;
    ecu_frame_t frame;
    while (1) {
        // Priorité aux OUTPUT du PID
        if (xQueueReceive(q_output, &output, 0) == pdTRUE) {
            build_frame(&frame, 0x80, &output, sizeof(float));
            xSemaphoreTake(mutex_uart_tx, portMAX_DELAY);
            uart_write_bytes(UART_NUM, frame.raw, frame.len);
            xSemaphoreGive(mutex_uart_tx);
        }
        // Puis les STATS et ALERT
        if (xQueueReceive(q_tx, &frame, pdMS_TO_TICKS(10)) == pdTRUE) {
            xSemaphoreTake(mutex_uart_tx, portMAX_DELAY);
            uart_write_bytes(UART_NUM, frame.raw, frame.len);
            xSemaphoreGive(mutex_uart_tx);
        }
    }
}
```

#### Liée à

- **PID** (reçoit `output`)
- **Stats** (reçoit trame STATS)
- **Failsafe** (reçoit trame ALERT)

---

### 5. Tâche Failsafe

**Priorité :** 5 — priorité maximale  
**Stack :** 1024 octets (petite, fait peu de choses)  
**Déclenchement :** Événementielle — watchdog 2 s + ISR GPIO

#### Rôle

Le gardien de sécurité. Elle surveille deux événements indépendants :

1. **Timeout 2 s :** aucune trame valide reçue depuis 2 secondes
2. **GPIO :** signal d'erreur externe sur une broche (bouton ou signal hardware)

Dans les deux cas, elle doit réagir en moins de 5 ms : couper le moteur (`output = 0`), passer en mode `OFF`, et envoyer immédiatement une trame `ALERT 0x85`.

#### Mécanisme watchdog

Le Parser appelle `xTaskNotify(failsafe_handle, ...)` à chaque trame valide reçue. La tâche Failsafe attend cette notification avec un timeout de 2 s. Si le timeout expire → personne n'a envoyé de trame depuis 2 s → déclenchement.

#### Entrées / Sorties

- **Entrées :**
  - `xTaskNotify` depuis Parser (reset watchdog)
  - Semaphore `sem_gpio` depuis ISR GPIO
- **Sorties :**
  - Mode forcé `OFF` + `output = 0` (via `mutex_vars`)
  - Queue `q_tx` → trame `ALERT 0x85`

#### Appels FreeRTOS

| Appel | Pourquoi |
|---|---|
| `ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000))` | Attend 2 s max. Si personne ne notifie → timeout = déclenchement failsafe |
| `xSemaphoreGiveFromISR(sem_gpio, &woken)` | L'ISR GPIO donne le semaphore depuis le contexte d'interruption |
| `portYIELD_FROM_ISR(woken)` | Force un changement de contexte immédiat si la tâche Failsafe est de plus haute priorité |
| `xSemaphoreTake(mutex_vars, portMAX_DELAY)` | Pour écrire `mode = OFF` et `output = 0` de façon sûre |
| `xQueueSend(q_tx, &alert_frame, 0)` | Envoie l'ALERT `0x85` immédiatement |

#### Squelette de code

```c
void task_failsafe(void *pv) {
    while (1) {
        uint32_t notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000));
        if (notified == 0) {
            // Timeout : aucune trame valide depuis 2s
            trigger_failsafe("TIMEOUT: no frame for 2s");
        }
        // Le GPIO est géré via ISR séparée (voir ci-dessous)
    }
}

// ISR GPIO — s'exécute hors contexte tâche
void IRAM_ATTR gpio_isr_handler(void *arg) {
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(sem_gpio, &woken);
    portYIELD_FROM_ISR(woken);
}

// Fonction déclenchement (appelée depuis task_failsafe ou une tâche GPIO)
void trigger_failsafe(const char *reason) {
    xSemaphoreTake(mutex_vars, portMAX_DELAY);
    mode   = OFF;
    output = 0.0f;
    xSemaphoreGive(mutex_vars);

    ecu_frame_t alert;
    build_frame(&alert, 0x85, (uint8_t*)reason, strlen(reason));
    xQueueSend(q_tx, &alert, 0);
}
```

#### Liée à

- **Parser** (reçoit la notification watchdog)
- **TX** (envoie l'ALERT)
- **GPIO ISR** (reçoit le semaphore)

---

### 6. Tâche Stats

**Priorité :** 1 — priorité minimale  
**Stack :** 2048 octets  
**Déclenchement :** Toutes les 1 seconde via `vTaskDelay`

#### Rôle

La tâche de monitoring. Toutes les secondes, elle lit les compteurs cumulatifs depuis le démarrage et construit une trame `STATS 0x83` qu'elle envoie dans la queue TX. Elle a la priorité la plus basse pour ne jamais gêner les tâches critiques.

#### Compteurs envoyés (dans l'ordre du payload)

| Index | Compteur | Description |
|---|---|---|
| 0 | `rx_valid` | Trames valides reçues |
| 1 | `rx_crc_err` | Trames rejetées (CRC invalide) |
| 2 | `rx_dropped` | Trames perdues (queue pleine) |
| 3 | `tx_output` | Messages OUTPUT émis |

#### Entrées / Sorties

- **Entrée :** Variables globales atomiques (compteurs `stats`)
- **Sortie :** Queue `q_tx` → trame `STATS 0x83`

#### Appels FreeRTOS

| Appel | Pourquoi |
|---|---|
| `vTaskDelay(pdMS_TO_TICKS(1000))` | Attend 1 seconde. Contrairement à `vTaskDelayUntil`, pas de contrainte stricte ici |
| `xQueueSend(q_tx, &frame, 0)` | Envoie les stats. `0` = non-bloquant, si queue pleine on skip cette seconde |

#### Squelette de code

```c
void task_stats(void *pv) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        uint32_t payload[4] = {
            stats.rx_valid,
            stats.rx_crc_err,
            stats.rx_dropped,
            stats.tx_output
        };

        ecu_frame_t frame;
        build_frame(&frame, 0x83, (uint8_t*)payload, sizeof(payload));
        xQueueSend(q_tx, &frame, 0);
    }
}
```

#### Liée à

- **TX** (envoie la trame STATS)

---

## Points clés pour le rapport

### vTaskDelayUntil vs vTaskDelay

C'est une différence fondamentale à mentionner dans le rapport :

- `vTaskDelay(100ms)` attend 100 ms **après la fin** de l'exécution. Si le code prend 2 ms, la période réelle est 102 ms.
- `vTaskDelayUntil(&last, 100ms)` garantit 100 ms **entre chaque début d'exécution**, peu importe le temps d'exécution.

`vTaskDelayUntil` est obligatoire pour la tâche PID qui a une contrainte stricte de jitter < 5 ms.

### Choix du timeout 0 sur le mutex dans le PID

```c
xSemaphoreTake(mutex_vars, 0); // timeout = 0 = non-bloquant
```

Si le Parser est en train d'écrire au même moment, le PID utilise les anciennes valeurs plutôt que de bloquer et rater sa fenêtre de 100 ms. C'est acceptable car la vitesse d'un véhicule physique ne change pas en 100 ms de façon significative.

### Isolation des défaillances

La séparation UART RX / Parser en deux tâches distinctes est cruciale : un flood de messages (50 trames corrompues d'un coup) remplit au pire la queue `q_bytes`, incrémente `rx_dropped`, mais **ne retarde jamais la tâche PID** qui tourne indépendamment à priorité 5.

### Anti-windup PID

Quand la sortie est saturée, on annule l'accumulation de l'intégrale pour éviter qu'elle explose :

```c
if (output > 255.0f) {
    output = 255.0f;
    if (err > 0) integral -= err * dt; // anti-windup
}
```

---

*Document généré pour le TP Conception d'un ECU Temps Réel — EPITA 2026*
