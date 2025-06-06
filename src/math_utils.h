#pragma once

#include <math.h>
#include <stdint.h>

// Scalar utils
float signf(float x);
float clampf(float x, float min, float max);
float maxf(float a, float b);
float minf(float a, float b);
uint16_t float_to_fixed(float v, float min, float max);
float fixed_to_float(uint16_t val, float min, float max);

// Vector algebra
void vec3_add(float result[3], const float a[3], const float b[3]);
void vec3_scalar_mul(float result[3], const float v[3], float scalar);
void vec3_outer_product(float result[3][3], const float a[3], const float b[3]);
float vec3_dot_product(const float a[3], const float b[3]);


// Matrix Algebra
void mat3x3_mul(float result[3][3], const float A[3][3], const float B[3][3]);
void mat3x3_vec_mul(float result[3], const float A[3][3], const float v[3]);
void mat3x3_transpose(float result[3][3], const float A[3][3]);
void matrix_add_3x3(float result[3][3], const float A[3][3], const float B[3][3]);
void matrix_sub_3x3(float result[3][3], const float A[3][3], const float B[3][3]);
void matrix_elementwise_mul_3x3(float result[3][3], const float A[3][3], const float B[3][3]);
void mat3x3_add_scalar(float result[3][3], const float A[3][3], const float scalar);
void mat3x3_mul_scalar(float result[3][3], const float A[3][3], const float scalar);
void mat3x3_eye(float result[3][3]);


// Ramps
float linear_ramp_to(float x, float step, float target);

// Time utils
float calc_dt_from_timestamps_us(uint32_t then, uint32_t now);
