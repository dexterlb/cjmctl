#pragma once

typedef struct {
    float vel_coast;

    float pos_gain;
} control_position_cfg_t;

typedef struct {
    control_position_cfg_t* cfg;

    // state
    float pos_err;
    float now;

    // params
    float pos_target;

    // inputs
    float pos_measured;

    // outputs
    float vel_output;
} control_position_t;

// WARNING: this library is unfinished and does not work! do not use yet!

void control_position_init(control_position_t* cpos, control_position_cfg_t* cfg);
void control_position_update(control_position_t* cpos, float dt);
void control_position_report_pos(control_position_t* cpos, float pos);
void control_position_target_pos(control_position_t* cpos, float pos);
