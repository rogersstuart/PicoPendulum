/*
 * Control Utility Functions
 *
 * This header defines a set of simple mathematical helpers used by
 * both the embedded controller and the PC simulator.  Providing these
 * functions in a separate common module promotes code sharing
 * between the two builds and reduces duplication.  All functions are
 * implemented as pure C to ease integration with C++ when included
 * from the simulator.  The functions operate on single precision
 * floating point values and are declared `static inline` to allow
 * efficient compilation on the Pico microcontroller.
 */

#ifndef COMMON_CONTROL_UTILS_H
#define COMMON_CONTROL_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

/* Clamp x to the range [lo, hi].  If x is less than lo return lo;
 * if greater than hi return hi; otherwise return x. */
static inline float ctl_clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

/* Return the sign of x: +1 if positive, -1 if negative, 0 if zero. */
static inline float ctl_sgn(float x) {
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

/* Wrap an angle into the range (−π, π].  This function repeatedly
 * subtracts or adds 2π until the argument lies within the desired
 * interval. */
static inline float ctl_wrap_pi(float x) {
    while (x > (float)M_PI)  x -= 2.0f * (float)M_PI;
    while (x <= -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* COMMON_CONTROL_UTILS_H */