#include "plant.h"

void plant_init(plant_t *p, float alpha) {
    p->y = 0.0f;
    p->alpha = alpha;
}

float plant_step(plant_t *p, float u) {
    float dy = p->alpha * (-p->y + u);
    p->y += dy;
    return p->y;
}