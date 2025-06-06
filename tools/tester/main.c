#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <stdint.h>
#include <err.h>
#include <math.h>
#include <string.h>

#include "estimator_velocity.h"

bool read_char(char* c) {
    int n = read(0, c, 1);
    if (n < 0) {
        err(1, "could not read from stdin");
    }
    return (n != 0);
}

void update_estimator(estimator_velocity_t* estimator, uint32_t timestamp, float input_pos) {
    static estimator_velocity_cfg_t cfg;

    if (estimator->cfg == NULL) {
        // first measurement
        cfg.init_pos = input_pos;
        cfg.init_vel = 0;
        cfg.init_acc = 0;
        cfg.process_noise_pos = 0.1f;
        cfg.process_noise_vel = 1000.0f;
        cfg.process_noise_acc = 2000.0f;
        cfg.measurement_noise_pos = 1.0f;
        cfg.vel_est_stride = INFINITY;
        cfg.max_possible_vel = INFINITY;
        estimator_velocity_init(estimator, &cfg);
    }

    estimator_velocity_update(estimator, input_pos, timestamp);
}

void process_line(const char* line, estimator_velocity_t* estimator) {
    float timestamp;
    float input_pos;

    sscanf(line, "%f %f", &timestamp, &input_pos);
    fprintf(stderr, "%f %f\n", timestamp, input_pos);
    update_estimator(estimator, timestamp, input_pos);
    dprintf(
        1, "%f %f %f %f\n",
        (float)timestamp / 1000000.0f,
        estimator->out.pos_estimate,
        estimator->out.vel_estimate,
        estimator->out.acc_estimate
    );
}

int main() {
    char line[1024];
    uint16_t i = 0;

    estimator_velocity_t estimator;
    memset(&estimator, 0, sizeof(estimator));

    while (read_char(&line[i])) {
        if (line[i] == '\n') {
            line[i] = '\0';
            i = 0;
            process_line(line, &estimator);
        } else {
            i++;
        }
    }
}
