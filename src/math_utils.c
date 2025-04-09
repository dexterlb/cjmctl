#include "math_utils.h"

#include <math.h>

// Scalar operations
float signf(float x) {
    if (x < 0) {
        return -1;
    }
    return 1;
}

float clampf(float x, float min, float max) {
    if (x < min) {
        return min;
    }
    if (x > max) {
        return max;
    }
    return x;
}

float minf(float a, float b) {
    if (a <= b) {
        return a;
    }
    return b;
}

float maxf(float a, float b) {
    if (a >= b) {
        return a;
    }
    return b;
}

// Ramps
float linear_ramp_to(float x, float step, float target) {
    step = fabs(step);
    float err = x - target;
    if (fabs(err) < step) {
        return target;
    }
    return x - step * signf(err);
}

// Vector algebra
void vec3_add(float result[3], const float a[3], const float b[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = a[i] + b[i];
    }
}

void vec3_scalar_mul(float result[3], const float v[3], float scalar) {
    for (int i = 0; i < 3; ++i) {
        result[i] = v[i] * scalar;
    }
}

void vec3_outer_product(float result[3][3], const float a[3], const float b[3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = a[i] * b[j];
        }
    }
}

// Matrix Algebra
void mat3x3_mul(float result[3][3], const float A[3][3], const float B[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

void mat3x3_vec_mul(float result[3], const float A[3][3], const float v[3]) {
    for (int i = 0; i < 3; ++i) {
        result[i] = 0.0f;
        for (int j = 0; j < 3; ++j) {
            result[i] += A[i][j] * v[j];
        }
    }
}

void mat3x3_transpose(float result[3][3], const float A[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[j][i] = A[i][j];
        }
    }
}

void matrix_add_3x3(float result[3][3], const float A[3][3], const float B[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
}

void matrix_sub_3x3(float result[3][3], const float A[3][3], const float B[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = A[i][j] - B[i][j];
        }
    }
}

void matrix_elementwise_mul_3x3(float result[3][3], const float A[3][3], const float B[3][3]) {
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = A[i][j] * B[i][j];
        }
    }
}

// Time utils
float calc_dt_from_timestamps_us(uint32_t then, uint32_t now) {
    uint32_t diff = now - then;
    if (diff > UINT32_MAX / 2) {
        // something is definitely wrong
        return -INFINITY;
    }
    return (float)diff * 0.000001;
}
