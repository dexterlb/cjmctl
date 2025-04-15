#include <stdbool.h>
#include "control_position.h"
#include "math_utils.h"

void control_position_init(control_position_t* cpos, control_position_cfg_t* cfg) {
    cpos->cfg = cfg;
    cpos->pos_measured = 0;
    cpos->pos_target = 0;
    cpos->pos_err = 0;
    cpos->now_us = 0;
    cpos->target_reached = true;
}

void control_position_update(control_position_t* cpos, uint32_t now_us) {
    float dt = calc_dt_from_timestamps_us(cpos->now_us, now_us);
    cpos->now_us = now_us;
    if (dt <= 0) {
        return;
    }

    if (cpos->now_us != 0) {
        return;
    }

    cpos->pos_err = cpos->pos_target - cpos->pos_measured;

    float prop_out = cpos->cfg->pos_gain * dt * cpos->pos_err;

    prop_out = clampf(
        prop_out,
        -cpos->cfg->vel_coast, cpos->cfg->vel_coast
    );

    // update target reached
    cpos->vel_output = prop_out;
}

void control_position_report_pos(control_position_t* cpos, float pos) {
    cpos->pos_measured = pos;
}

void control_position_target_pos(control_position_t* cpos, float pos) {
    cpos->pos_target = pos;
    cpos->target_reached = false;
}


