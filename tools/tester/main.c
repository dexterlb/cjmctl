#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>
#include <err.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>

#include "estimator_velocity.h"

bool read_char(char* c) {
	int n = read(0, c, 1);
	if (n < 0) {
		err(1, "could not read from stdin");
	}
	return (n != 0);
}

void update_estimator(estimator_velocity_t* estimator, uint32_t timestamp_us, float input_pos) {
	static estimator_velocity_cfg_t cfg;

	if (estimator->cfg == NULL) {
		// First measurement, initialize the estimator
		cfg.init_pos              = input_pos;
		cfg.init_vel              = 0.0f;
		cfg.init_acc              = 0.0f;
		cfg.process_noise_pos     = 0.1f;
		cfg.process_noise_vel     = 1000.0f;
		cfg.process_noise_acc     = 2000.0f;
		cfg.measurement_noise_pos = 1.0f;
		cfg.max_possible_vel      = 10;
		estimator_velocity_init(estimator, &cfg);

		dprintf(2, "Vector X:\n");
		dprintf(2, "est pos %.4f: \n", estimator->out.pos_estimate);
		dprintf(2, "est vel %.4f: \n", estimator->out.vel_estimate);
		dprintf(2, "est acc %.4f: \n", estimator->out.acc_estimate);
		dprintf(2, "\n");
	}

	estimator_velocity_update(estimator, input_pos, timestamp_us);
}

void process_line(const char* line, estimator_velocity_t* estimator) {
	float timestamp_sec;
	float input_pos;

	sscanf(line, "%f %f", &timestamp_sec, &input_pos);
	// dprintf(2, "Input: %f %f\n", timestamp_sec, input_pos);

	update_estimator(estimator, (uint32_t)(timestamp_sec * 1e6), input_pos);

	dprintf(
		1, "%f %f %f %f\n",
		timestamp_sec,
		estimator->out.pos_estimate,
		estimator->out.vel_estimate,
		estimator->out.acc_estimate
	);
}

int main() {
	char     line[1024];
	uint16_t i = 0;

	estimator_velocity_t estimator;
	memset(&estimator, 0, sizeof(estimator));

	while (read_char(&line[i])) {
		if (line[i] == '\n') {
			line[i] = '\0';
			i       = 0;
			process_line(line, &estimator);
		} else {
			i++;
		}
	}

	return 0;
}
