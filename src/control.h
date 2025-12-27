#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>

// Maszyna stanów (FSM)
typedef enum {
    STATE_IDLE,
    STATE_RUN_OPEN,
    STATE_RUN_CLOSED,
    STATE_SAFE,
    STATE_FAULT
} system_state_t;

typedef struct {
    // Parametry PID i limity
    float kp, ki, kd;
    float u_min, u_max;
    float i_limit;
    int   wd_limit_ticks;

    // Stan wewnętrzny
    system_state_t state;
    float setpoint;     // Cel dla CLOSED
    float manual_out;   // Cel dla OPEN
    float u;            // Wyjście sterujące
    float i_term;       // Akumulator całki
    float prev_err;     // Poprzedni błąd (dla D)
    
    // Telemetria / Bezpieczeństwo
    int wd_counter;
    int wd_resets;
    bool tick_executed; // Flaga watchdoga
} control_t;

void ctrl_init(control_t *c);
void ctrl_config(control_t *c, float kp, float ki, float kd, float limit);
void ctrl_set_mode(control_t *c, system_state_t new_state);
void ctrl_set_point(control_t *c, float sp);
void ctrl_set_out(control_t *c, float out);

// Główna pętla sterowania (np. co 10ms)
void ctrl_tick(control_t *c, float measurement);

// Watchdog (np. wywoływany niezależnie co 1ms/10ms)
void ctrl_check_wd(control_t *c);

#endif