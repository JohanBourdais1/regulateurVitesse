#pragma once

typedef struct {
    float kp;
    float ki;
    float kd;
    float dt;
    float out_min;
    float out_max;
    float integral;
    float prev_err;
} pid_t;

/**
 * Initialise un contrôleur PID.
 */
void pid_init(pid_t *pid, float kp, float ki, float kd,
              float dt, float out_min, float out_max);

/**
 * Calcule une itération PID.
 * @param pid       contexte PID
 * @param setpoint  valeur cible
 * @param measured  valeur mesurée
 * @return          commande saturée entre out_min et out_max
 */
float pid_compute(pid_t *pid, float setpoint, float measured);

/**
 * Remet l'intégrale et l'erreur précédente à zéro.
 * À appeler lors d'une transition vers le mode AUTO.
 */
void pid_reset(pid_t *pid);
