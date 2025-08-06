
#ifndef FILTERS_H
#define FILTERS_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float y;
    float a; // 0..1, smaller = more smoothing
    int initialized;
} lpf1_t;

static inline void lpf1_init(lpf1_t *f, float alpha, float y0) { f->a = alpha; f->y = y0; f->initialized = 0; }
static inline float lpf1_step(lpf1_t *f, float x) {
    if (!f->initialized) { f->y = x; f->initialized = 1; return f->y; }
    f->y = f->a * f->y + (1.0f - f->a) * x;
    return f->y;
}

typedef struct {
    float x1, y1;
    float a; // smoothing for derivative
    int initialized;
} diff_lpf_t;

static inline void diff_lpf_init(diff_lpf_t *d, float alpha) { d->x1 = 0.f; d->y1 = 0.f; d->a = alpha; d->initialized = 0; }
static inline float diff_lpf_step(diff_lpf_t *d, float x, float dt) {
    if (!d->initialized) { d->x1 = x; d->y1 = 0.f; d->initialized = 1; return 0.f; }
    float dx = (x - d->x1) / (dt > 1e-6f ? dt : 1e-6f);
    d->x1 = x;
    // low-pass the derivative
    d->y1 = d->a * d->y1 + (1.0f - d->a) * dx;
    return d->y1;
}

// 2-state discrete Kalman filter for [theta; omega] with measurement z = theta
typedef struct {
    // State transition matrix elements
    float A11, A12, A21, A22;
    // Input matrix elements
    float B1, B2;
    // Measurement matrix elements
    float C1, C2;
    // Process noise covariance diagonals
    float Q11, Q22;
    // Measurement noise covariance
    float R;
    // State estimates [theta; omega]
    float state_theta, state_omega;
    // Error covariance matrix elements
    float P11, P12, P21, P22;
} KalmanFilter;

static inline void kalman_init(KalmanFilter *k, float dt, float a, float b, float q_theta, float q_omega, float r_meas) {
    // xdot = [0 1; a 0] x + [0; b] u  (linearized about upright)
    // Discretize (Euler) for simplicity
    k->A11 = 1.0f;         k->A12 = dt;
    k->A21 = a * dt;       k->A22 = 1.0f;
    k->B1 = 0.0f;          k->B2 = b * dt;
    k->C1 = 1.0f;          k->C2 = 0.0f;
    k->Q11 = q_theta;      k->Q22 = q_omega;
    k->R   = r_meas;
    k->state_theta = 0.f;  k->state_omega = 0.f;
    k->P11 = 1e-3f;        k->P12 = 0.f; 
    k->P21 = 0.f;          k->P22 = 1e-3f;
}

static inline void kalman_predict(KalmanFilter *k, float u) {
    // x = A x + B u
    float next_theta = k->A11 * k->state_theta + k->A12 * k->state_omega + k->B1 * u;
    float next_omega = k->A21 * k->state_theta + k->A22 * k->state_omega + k->B2 * u;
    // P = A P A' + Q
    float P11 = k->A11*k->P11 + k->A12*k->P21;
    float P12 = k->A11*k->P12 + k->A12*k->P22;
    float P21 = k->A21*k->P11 + k->A22*k->P21;
    float P22 = k->A21*k->P12 + k->A22*k->P22;
    float nP11 = P11*k->A11 + P12*k->A12 + k->Q11;
    float nP12 = P11*k->A21 + P12*k->A22;
    float nP21 = P21*k->A11 + P22*k->A12;
    float nP22 = P21*k->A21 + P22*k->A22 + k->Q22;
    k->state_theta = next_theta; k->state_omega = next_omega;
    k->P11 = nP11; k->P12 = nP12; k->P21 = nP21; k->P22 = nP22;
}

static inline void kalman_update(KalmanFilter *k, float z) {
    // y = z - C x
    float y = z - (k->C1 * k->state_theta + k->C2 * k->state_omega);
    // S = C P C' + R
    float S = k->P11 + k->R;
    // K = P C'/S
    float K1 = k->P11 / S;
    float K2 = k->P21 / S;
    // x = x + K y
    k->state_theta += K1 * y;
    k->state_omega += K2 * y;
    // P = (I - K C) P
    float P11 = (1.0f - K1) * k->P11;
    float P12 = (1.0f - K1) * k->P12;
    float P21 = -K2 * k->P11 + k->P21;
    float P22 = -K2 * k->P12 + k->P22;
    k->P11 = P11; k->P12 = P12; k->P21 = P21; k->P22 = P22;
}

// Getter functions for state variables
static inline float kalman_get_theta(const KalmanFilter *k) { return k->state_theta; }
static inline float kalman_get_omega(const KalmanFilter *k) { return k->state_omega; }

#ifdef __cplusplus
}
#endif

#endif
