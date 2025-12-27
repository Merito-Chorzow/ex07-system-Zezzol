#include "control.h"
#include <stdio.h>

static float constrain(float v, float min, float max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}

void ctrl_init(control_t *c) {
    c->state = STATE_IDLE;
    c->kp = 1.0f; c->ki = 0.0f; c->kd = 0.0f;
    c->u_min = -100.0f; c->u_max = 100.0f;
    c->i_limit = 100.0f;
    c->wd_limit_ticks = 5; // Próg watchdoga (liczba pominiętych ticków)
    
    c->setpoint = 0.0f; c->manual_out = 0.0f; c->u = 0.0f;
    c->i_term = 0.0f; c->prev_err = 0.0f;
    c->wd_counter = 0; c->wd_resets = 0;
    c->tick_executed = false;
}

void ctrl_config(control_t *c, float kp, float ki, float kd, float limit) {
    c->kp = kp; c->ki = ki; c->kd = kd;
    c->u_min = -limit; c->u_max = limit;
    c->i_limit = limit;
}

void ctrl_set_mode(control_t *c, system_state_t new_state) {
    if (c->state == STATE_SAFE) return; // Z SAFE można wyjść tylko resetem (uproszczenie)
    
    if (c->state != new_state) {
        // Bumpless transfer: inicjalizacja całki aktualnym wyjściem przy wejściu w CLOSED
        if (new_state == STATE_RUN_CLOSED) {
            c->i_term = c->u; 
            c->i_term = constrain(c->i_term, -c->i_limit, c->i_limit);
            c->prev_err = 0;
        }
        c->state = new_state;
    }
}

void ctrl_set_point(control_t *c, float sp) { c->setpoint = sp; }
void ctrl_set_out(control_t *c, float out)  { c->manual_out = out; }

void ctrl_tick(control_t *c, float measurement) {
    c->tick_executed = true; // Zgłoszenie "życia" do watchdoga
    
    if (c->state == STATE_SAFE || c->state == STATE_FAULT || c->state == STATE_IDLE) {
        c->u = 0.0f;
        return;
    }

    if (c->state == STATE_RUN_OPEN) {
        // Tu można dodać rampę (slew-rate limit)
        c->u = constrain(c->manual_out, c->u_min, c->u_max);
    } 
    else if (c->state == STATE_RUN_CLOSED) {
        float error = c->setpoint - measurement;
        
        // PID
        float p_term = c->kp * error;
        c->i_term += c->ki * error;
        c->i_term = constrain(c->i_term, -c->i_limit, c->i_limit); // Anti-windup
        float d_term = c->kd * (error - c->prev_err);
        c->prev_err = error;
        
        c->u = constrain(p_term + c->i_term + d_term, c->u_min, c->u_max);
    }
}

void ctrl_check_wd(control_t *c) {
    if (c->state == STATE_SAFE) return;

    if (c->tick_executed) {
        c->wd_counter = 0;
        c->tick_executed = false; // Reset flagi, czekamy na kolejny tick
    } else {
        c->wd_counter++;
        if (c->wd_counter >= c->wd_limit_ticks) {
            c->state = STATE_SAFE;
            c->u = 0.0f;
            c->wd_resets++;
            printf("[WD] Watchdog timeout! Przejście do SAFE.\n");
        }
    }
}