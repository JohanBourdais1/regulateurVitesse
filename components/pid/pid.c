#include "pid.h"
#include <math.h>

void pid_init(pid_t *pid, float kp, float ki, float kd,
              float dt, float out_min, float out_max)
{
    pid->kp       = kp;
    pid->ki       = ki;
    pid->kd       = kd;
    pid->dt       = dt;
    pid->out_min  = out_min;
    pid->out_max  = out_max;
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
}

float pid_compute(pid_t *pid, float setpoint, float measured)
{
    float err = setpoint - measured;

    // Terme intégral (candidat avant saturation)
    float integral_candidate = pid->integral + err * pid->dt;

    // Terme dérivé
    float derivative = (err - pid->prev_err) / pid->dt;

    // Sortie brute
    float output = (pid->kp * err)
                 + (pid->ki * integral_candidate)
                 + (pid->kd * derivative);

    // Saturation + anti-windup conditionnel
    if (output > pid->out_max) {
        output = pid->out_max;
        // Anti-windup : on n'accumule pas si l'erreur pousse dansle même sens que la saturation
        if (err > 0.0f)
            integral_candidate = pid->integral;
    } else if (output < pid->out_min) {
        output = pid->out_min;
        if (err < 0.0f)
            integral_candidate = pid->integral;
    }

    pid->integral = integral_candidate;
    pid->prev_err = err;

    return output;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
}
