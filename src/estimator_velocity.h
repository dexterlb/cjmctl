#pragma once

#include <stdint.h>
#include <stdbool.h>

#define estimator_velocity_num_prev_positions 50

#include <stdio.h>

typedef struct {
    float init_pos;
    float init_vel;
    float init_acc;
    float default_process_variance;
    float default_measurement_variance;
} estimator_velocity_cfg_t;

typedef struct {
    estimator_velocity_cfg_t* cfg;

    union {
        float x[3];
        struct {
            float pos_estimate;
            float vel_estimate;
            float acc_estimate;
        } out;
    };
    float state[3][3];

    uint32_t now_us;
} estimator_velocity_t;

void estimator_velocity_init(estimator_velocity_t* est, estimator_velocity_cfg_t* cfg);


// All of the following functions do the same, they just take parameters differently
void estimator_velocity_update_dflt(estimator_velocity_t *est, float measured_pos, uint32_t now_us);
void estimator_velocity_update(estimator_velocity_t *est, float measured_pos, uint32_t now_us, float measurement_var, float process_var);

void estimator_velocity_update_dt_dflt(estimator_velocity_t *est, float measured_pos, float dt);
void estimator_velocity_update_dt(estimator_velocity_t *est, float measured_pos, float dt, float measurement_var, float process_var);
