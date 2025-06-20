#include "estimator_velocity.h"
#include "math_utils.h"

void estimator_velocity_init(estimator_velocity_t* est, estimator_velocity_cfg_t* cfg) {
	est->cfg = cfg;

	est->x[0] = cfg->init_pos;
	est->x[1] = cfg->init_vel;
	est->x[2] = cfg->init_acc;

	mat3x3_eye(est->state);

	est->state[0][0] = cfg->measurement_noise_pos;

	est->now_us = UINT32_MAX;
}

void estimator_velocity_update(estimator_velocity_t* est, float measured_pos, uint32_t now_us) {
	float dt    = calc_dt_from_timestamps_us(est->now_us, now_us);
	est->now_us = now_us;
	if (dt <= 0) {
		return;
	}

	estimator_velocity_update_dt(est, measured_pos, dt);
}

void estimator_velocity_update_dt(estimator_velocity_t* est, float measured_pos, float dt) {
	float state_transition[3][3] = {
		// position, velocity and acceleration model
		// pos = v0 * t + 0.5 * a * t^2
		{1, dt, 0.5f * dt * dt},
		{0, 1, dt},
		{0, 0, 1}
	};

	float x_pred[3];
	mat3x3_vec_mul(x_pred, state_transition, est->x);

	float covariance_pred[3][3];
	mat3x3_mul(covariance_pred, state_transition, est->state);

	float state_transition_transp[3][3]; // I hope the optimiser makes this in-place
	mat3x3_transpose(state_transition_transp, state_transition);

	float state_covariance_pred[3][3];
	mat3x3_mul(state_covariance_pred, covariance_pred, state_transition_transp);

	float process_noise_covariance[3][3] = {
		{est->cfg->process_noise_pos, 0, 0},
		{0, est->cfg->process_noise_vel, 0},
		{0, 0, est->cfg->process_noise_acc}
	};

	matrix_add_3x3(state_covariance_pred, state_covariance_pred, process_noise_covariance);

	float H[3] = {1, 0, 0};
	float y    = measured_pos - vec3_dot_product(H, x_pred);

	float S;
	float tmp_v[3];
	mat3x3_vec_mul(tmp_v, state_covariance_pred, H); //

	S = vec3_dot_product(H, tmp_v);
	S += est->cfg->measurement_noise_pos;

	float K[3];
	vec3_scalar_mul(K, tmp_v, 1 / S);

	float offset_v[3];
	vec3_scalar_mul(offset_v, K, y);
	vec3_add(est->x, x_pred, offset_v);

	float KH[3][3];
	vec3_outer_product(KH, K, H);

	float KH_neg[3][3];
	mat3x3_mul_scalar(KH_neg, KH, -1);

	float eye_matrix[3][3];
	mat3x3_eye(eye_matrix);

	float state_update_matrix[3][3];

	matrix_add_3x3(state_update_matrix, eye_matrix, KH_neg);
	mat3x3_mul(est->state, state_update_matrix, state_covariance_pred);

	est->out.vel_estimate = clampf(est->out.vel_estimate, -est->cfg->max_possible_vel, est->cfg->max_possible_vel);
}
