#include "estimator_velocity_dumb.h"

#include <stdio.h>
#include <math.h>

void estimator_velocity_dumb_init(estimator_velocity_dumb_t* est, estimator_velocity_dumb_cfg_t* cfg) {
    est->cfg = cfg;
    est->last_pos = estimator_velocity_dumb_num_prev_positions - 1;
    est->num_positions = 0;
    est->reliable = false;
}

void estimator_velocity_dumb_report_pos_reading(estimator_velocity_dumb_t* est, float pos, float now) {
    uint8_t maxn = estimator_velocity_dumb_num_prev_positions;

    est->last_pos = (est->last_pos + 1) % maxn;
    est->positions[est->last_pos] = pos;
    est->timestamps[est->last_pos] = now;
    if (est->num_positions < maxn) {
        est->num_positions++;
        est->vel_estimate = 0;
        est->reliable = false;
        return;
    } else {
        est->reliable = true;
    }

    uint8_t first_pos = ((maxn + est->last_pos - est->num_positions + 1) % maxn);

    est->vel_estimate =
        (est->positions[est->last_pos] - est->positions[first_pos]) /
        (est->timestamps[est->last_pos] - est->timestamps[first_pos]);
    if (est->vel_estimate > 1000) {
        dprintf(1, "wtf?\n");
    }
}
