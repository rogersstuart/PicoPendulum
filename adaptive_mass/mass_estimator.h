/*
 * Adaptive Mass Estimator
 *
 * This module implements a simple recursive least squares (RLS) estimator
 * for the pendulum mass.  It is designed to run identically on both the
 * PC simulator and the embedded Pico platform.  The estimator treats the
 * pendulum dynamics as linear in the unknown mass and updates the mass
 * estimate on every control cycle.  A forgetting factor is used to
 * gracefully track slow changes in the system (e.g. wear or small loads)
 * while still converging quickly on startup.  Basic tamper detection is
 * included by monitoring the residual error: if the model error remains
 * consistently large the estimator flags a tamper condition which can be
 * used to transition the control system into a fault state.
 */

#ifndef ADAPTIVE_MASS_MASS_ESTIMATOR_H
#define ADAPTIVE_MASS_MASS_ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/*
 * Structure holding the state of the mass estimator.  The estimator is
 * deliberately simple to avoid excessive resource consumption on the
 * Pico microcontroller while still being useful on the PC.  It tracks
 * the current mass estimate, the covariance of the estimate, the
 * forgetting factor, bounds on the estimate, a previous velocity for
 * computing angular acceleration, and simple statistics for tamper
 * detection.
 */
typedef struct {
    float m_est;          /* Current mass estimate (kg) */
    float P;              /* Error covariance (kg^2) */
    float lambda;         /* Forgetting factor (0<lambda<=1) */
    float m_min;          /* Minimum allowable mass estimate (kg) */
    float m_max;          /* Maximum allowable mass estimate (kg) */
    float prev_omega;     /* Previous angular velocity (rad/s) */
    float error_avg;      /* Exponential moving average of residual error */
    float error_var;      /* Exponential moving average of squared residual error */
    int   tamper_count;   /* Counter for consecutive large residuals */
    bool  tampered;       /* Flag indicating a tamper condition */
    bool  initialized;    /* Has the estimator been initialised? */
} mass_estimator_t;

/* Initialise the mass estimator.  Pass in an initial mass guess.  If
 * zero or negative, a conservative default will be used.  The error
 * covariance is initialised to a relatively large value to allow for
 * rapid initial adaptation.  The forgetting factor, bounds and
 * detection thresholds are chosen based on typical pendulum systems but
 * can be tweaked by modifying the constants in mass_estimator.c.
 */
void mass_estimator_init(mass_estimator_t *est, float initial_mass);

/* Reset the estimator to its uninitialised state.  Useful if the
 * physical pendulum is replaced or if gross tampering has been
 * detected.  After reset the next call to mass_estimator_update() will
 * reinitialise with a new mass guess.
 */
void mass_estimator_reset(mass_estimator_t *est);

/* Perform a single RLS update of the mass estimator.
 *
 * Parameters:
 *  - est      : pointer to the estimator state
 *  - tau      : applied torque (N·m).  This should be the torque
 *               applied by the motor after any saturation or inversion.
 *  - theta    : pendulum angle from upright (rad).  Positive when the
 *               pendulum leans to the right.  Used in the gravity term.
 *  - omega    : current angular velocity (rad/s)
 *  - dt       : control timestep (s)
 *  - length   : pendulum length (m)
 *  - Jm       : motor/hub inertia about the axis (kg·m²)
 *  - b_vis    : viscous friction coefficient (N·m·s)
 *  - tau_ff   : Coulomb friction magnitude (N·m).  The sign is applied
 *               internally based on omega.
 *
 * Returns the updated mass estimate.  This value is already bounded
 * between m_min and m_max.  Tamper detection is also updated.
 */
float mass_estimator_update(mass_estimator_t *est,
                            float tau,
                            float theta,
                            float omega,
                            float dt,
                            float length,
                            float Jm,
                            float b_vis,
                            float tau_ff);

/* Get the current mass estimate from the estimator.  This simply
 * returns the last stored estimate.  It is provided for API
 * completeness.
 */
static inline float mass_estimator_get_mass(const mass_estimator_t *est) {
    return est->m_est;
}

/* Determine whether a tamper condition has been detected.  When true,
 * the control system should transition into a fault state and prevent
 * further motion until the system is inspected.  The condition clears
 * automatically when the estimator is reset.  A tamper condition is
 * raised when the residual error remains above a threshold for a
 * sustained period of time.
 */
static inline bool mass_estimator_is_tampered(const mass_estimator_t *est) {
    return est->tampered;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ADAPTIVE_MASS_MASS_ESTIMATOR_H */