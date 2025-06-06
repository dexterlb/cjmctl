#pragma once

#include <stdint.h>
#include <stdbool.h>

#define estimator_velocity_num_prev_positions 50

#include <stdio.h>

typedef struct {
    float init_pos;
    float init_vel;
    float init_acc;
    float process_noise_pos;
    float process_noise_vel;
    float process_noise_acc;
    float measurement_noise_pos;
    float vel_est_stride;
    float max_possible_vel;
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

    float vel_est_min;
    float vel_est_max;
    uint32_t now_us;
} estimator_velocity_t;

void estimator_velocity_init(estimator_velocity_t* est, estimator_velocity_cfg_t* cfg);

void estimator_velocity_update(estimator_velocity_t *est, float measured_pos, uint32_t now_us);
void estimator_velocity_update_dt(estimator_velocity_t *est, float measured_pos, float dt);

void print_matrix_3x3(const float mat[3][3]);