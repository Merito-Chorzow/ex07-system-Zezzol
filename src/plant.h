#ifndef PLANT_H
#define PLANT_H

typedef struct {
    float y;
    float alpha; // Współczynnik bezwładności (0.0 - 1.0)
} plant_t;

// Inicjalizacja: alpha ~ Ts/Tau
void plant_init(plant_t *p, float alpha);

// Krok symulacji: y[k+1] = y[k] + alpha*(-y[k] + u[k])
float plant_step(plant_t *p, float u);

#endif