#include "estimator_velocity.h"

#include <stdio.h>
#include "math_utils.h"

void estimator_velocity_init(estimator_velocity_t *est, estimator_velocity_cfg_t* cfg) {
    est->cfg = cfg;

    est->x[0] = cfg->init_pos;
    est->x[1] = cfg->init_vel;
    est->x[2] = cfg->init_acc;

    est->vel_est_max = cfg->init_vel;
    est->vel_est_min = cfg->init_vel;
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (i == j) {
                est->state[i][j] = 1;
            } else {
                est->state[i][j] = 0;
            }
        }
    }

    est->state[0][0] = cfg->default_measurement_variance;

    est->now_us = UINT32_MAX;
}

void estimator_velocity_update_dflt(estimator_velocity_t *est, float measured_pos, uint32_t now_us) {
    float dt = calc_dt_from_timestamps_us(est->now_us, now_us);
    est->now_us = now_us;
    if (dt <= 0) {
        return;
    }

    estimator_velocity_update_dt_dflt(est, measured_pos, dt);
}

void estimator_velocity_update(estimator_velocity_t *est, float measured_pos, uint32_t now_us, float measurement_var, float process_var) {
    float dt = calc_dt_from_timestamps_us(est->now_us, now_us);
    est->now_us = now_us;
    if (dt <= 0) {
        return;
    }

    estimator_velocity_update_dt(est, measured_pos, dt, measurement_var, process_var);
}

void estimator_velocity_update_dt_dflt(estimator_velocity_t *est, float measured_pos, float dt) {
    estimator_velocity_update_dt(est, measured_pos, dt, est->cfg->default_measurement_variance, est->cfg->default_process_variance);
}

void estimator_velocity_update_dt(estimator_velocity_t *est, float measured_pos, float dt, float measurement_var, float process_var) {
    float state_transition[3][3] = {
        // position, velocity and acceleration model
        // pos = v0 * t + 0.5 * a * t^2
        {1,     dt,      0.5f * dt * dt},
        {0,      1,                  dt},
        {0,      0,                   1}
    };

    float x_pred[3];
    mat3x3_vec_mul(x_pred, state_transition, est->x);

    float covariance_pred[3][3];
    mat3x3_mul(covariance_pred, state_transition, est->state);

    float state_transition_transp[3][3];    // I hope the optimiser makes this in-place
    mat3x3_transpose(state_transition_transp, state_transition);

    float state_covariance_pred[3][3];
    mat3x3_mul(state_covariance_pred, covariance_pred, state_transition_transp);

    float noise_covariance[3][3] = {
        {process_var, 0, 0},
        {0, process_var, 0},
        {0, 0, process_var}
    };

    matrix_add_3x3(state_covariance_pred, state_covariance_pred, noise_covariance);

    float H[3] = {1, 0, 0};
    float y = measured_pos - x_pred[0];
    float meas_covariance = state_covariance_pred[0][0] + measurement_var;

    float gain[3] = {
        state_covariance_pred[0][0] / meas_covariance,
        state_covariance_pred[1][0] / meas_covariance,
        state_covariance_pred[2][0] / meas_covariance
    };

    float contribution[3];
    vec3_scalar_mul(contribution, gain, y);
    vec3_add(est->x, x_pred, contribution);

    float KH[3][3];
    vec3_outer_product(KH, gain, H);

    float KH_P[3][3];
    matrix_elementwise_mul_3x3(KH_P, KH, state_covariance_pred);

    matrix_sub_3x3(est->state, state_covariance_pred, KH_P);

    if(est->out.vel_estimate < est->vel_est_min)
    {
        est->vel_est_min = est->out.vel_estimate;
    } else {
        est->vel_est_min = linear_ramp_to(est->vel_est_min, est->cfg->vel_est_stride * dt, est->out.vel_estimate); 
    }

    est->out.vel_estimate = clampf(est->out.vel_estimate, -est->cfg->max_possible_vel, est->cfg->max_possible_vel);

    if(est->out.vel_estimate > est->vel_est_max)
    {
        est->vel_est_max = est->out.vel_estimate;
    } else {
        est->vel_est_max = linear_ramp_to(est->vel_est_max, est->cfg->vel_est_stride * dt, est->out.vel_estimate); 
    }
}
