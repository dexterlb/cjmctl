#include <stdbool.h>
#include "control_position.h"
#include "math_utils.h"

void control_position_check_target_reached(control_position_t* cpos);

void control_position_init(control_position_t* cpos, control_position_cfg_t* cfg) {
    cpos->cfg = cfg;
    cpos->pos_measured = 0;
    cpos->pos_target = 0;
    cpos->pos_err = 0;
    cpos->now_us = 0;
    cpos->target_reached = true;
    cpos->target_reached_timestamp = 0;
}

void control_position_update(control_position_t* cpos, uint32_t now_us) {
    float dt = calc_dt_from_timestamps_us(cpos->now_us, now_us);
    cpos->now_us = now_us;
    if (dt <= 0) {
        return;
    }

    if (!isnormal(dt)) {
        return;
    }

    if (cpos->target_reached) {
        cpos->vel_output = 0;
        return;
    }

    cpos->pos_err = cpos->pos_target - cpos->pos_measured;

    float prop_out = cpos->cfg->pos_gain * cpos->pos_err;

    prop_out = clampf(
        prop_out,
        -cpos->cfg->vel_coast, cpos->cfg->vel_coast
    );

    control_position_check_target_reached(cpos);
    cpos->vel_output = prop_out;

    if (fabs(cpos->pos_err) > cpos->cfg->vel_min_window && fabs(cpos->vel_output) < cpos->cfg->vel_min) {
        cpos->vel_output = cpos->cfg->vel_min * signf(cpos->vel_output);
    }
}

void control_position_report_pos(control_position_t* cpos, float pos) {
    cpos->pos_measured = pos;
}

void control_position_target_pos(control_position_t* cpos, float pos) {
    cpos->pos_target = pos;
    cpos->target_reached = false;
}

void control_position_check_target_reached(control_position_t* cpos) {
    float window = cpos->cfg->target_reached_window;
    if (window == 0.0f) {
        // user requested to never testify that target is reached
        return;
    }

    if (fabs(cpos->pos_measured - cpos->pos_target) >= window) {
        // we aren't close enough to the target
        cpos->target_reached_timestamp = cpos->now_us;
        return;
    }

    if (cpos->now_us - cpos->target_reached_timestamp <= cpos->cfg->target_reached_time_us) {
        // we haven't yet spent enough time in the "reached" window
        return;
    }

    cpos->target_reached = true;
}
