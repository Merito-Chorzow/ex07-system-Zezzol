#include <stdio.h>
#include "control.h"
#include "plant.h"

control_t ctrl;
plant_t plant;

const char* str_state(system_state_t s) {
    switch(s) {
        case STATE_IDLE: return "IDLE";
        case STATE_RUN_OPEN: return "OPEN";
        case STATE_RUN_CLOSED: return "CLOSED";
        case STATE_SAFE: return "SAFE";
        default: return "???";
    }
}

void sim_step(bool run_pid) {
    if (run_pid) ctrl_tick(&ctrl, plant.y); // Wykonaj tick regulatora
    ctrl_check_wd(&ctrl);                   // Sprawdź watchdog
    plant_step(&plant, ctrl.u);             // Fizyka obiektu działa zawsze
}

int main() {
    printf("=== Test A: Scenariusz Funkcjonalny ===\n");
    ctrl_init(&ctrl);
    ctrl_config(&ctrl, 2.0f, 0.1f, 0.0f, 10.0f); // Kp=2, Ki=0.1
    plant_init(&plant, 0.1f);

    // 1. OPEN MODE
    printf("[CMD] Tryb OPEN, Out=5.0\n");
    ctrl_set_mode(&ctrl, STATE_RUN_OPEN);
    ctrl_set_out(&ctrl, 5.0f);
    for(int i=0; i<10; i++) sim_step(true);
    printf("  [OPEN] y=%.2f u=%.2f\n", plant.y, ctrl.u);

    // 2. CLOSED MODE
    printf("[CMD] Tryb CLOSED, Setpoint=8.0 (Bumpless test)\n");
    ctrl_set_mode(&ctrl, STATE_RUN_CLOSED);
    ctrl_set_point(&ctrl, 8.0f);
    for(int i=0; i<20; i++) sim_step(true);
    printf("  [CLOSED] y=%.2f u=%.2f\n", plant.y, ctrl.u);

    // 3. STOP
    printf("[CMD] STOP -> IDLE\n");
    ctrl_set_mode(&ctrl, STATE_IDLE);
    sim_step(true);
    printf("  [IDLE] u=%.2f\n", ctrl.u);

    printf("\n=== Test C: Awaria (Watchdog) ===\n");
    ctrl_set_mode(&ctrl, STATE_RUN_CLOSED);
    ctrl_set_point(&ctrl, 5.0f);
    sim_step(true); // Normalny cykl
    printf("[Stan] %s, u=%.2f\n", str_state(ctrl.state), ctrl.u);
    
    printf("[SIM] Blokada funkcji tick()...\n");
    for(int i=1; i<=6; i++) {
        sim_step(false); // FALSE = brak wywołania ctrl_tick()
        printf("  [Tick symulacji %d] Stan=%s WD=%d\n", i, str_state(ctrl.state), ctrl.wd_counter);
    }

    return 0;
}