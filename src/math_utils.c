#include "math_utils.h"

#include <math.h>

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

float linear_ramp_to(float x, float step, float target) {
    step = fabs(step);
    float err = x - target;
    if (fabs(err) < step) {
        return target;
    }
    return x - step * signf(err);
}
