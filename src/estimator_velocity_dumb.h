#pragma once

#include <stdint.h>
#include <stdbool.h>

#define estimator_velocity_dumb_num_prev_positions 3

typedef struct {
    uint8_t something;
} estimator_velocity_dumb_cfg_t;

typedef struct {
    estimator_velocity_dumb_cfg_t* cfg;

    // internal state
    float positions[estimator_velocity_dumb_num_prev_positions];
    float timestamps[estimator_velocity_dumb_num_prev_positions];
    uint8_t last_pos;   // index of last item in positions
    uint8_t num_positions;

    // output
    float vel_estimate;
    bool reliable;
} estimator_velocity_dumb_t;

void estimator_velocity_dumb_init(estimator_velocity_dumb_t* est, estimator_velocity_dumb_cfg_t* cfg);
void estimator_velocity_dumb_report_pos_reading(estimator_velocity_dumb_t* est, float pos, float now);
