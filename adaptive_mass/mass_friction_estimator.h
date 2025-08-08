/*
 * Adaptive Mass & Friction Estimator
 *
 * This module implements a two‑parameter recursive least squares
 * estimator for simultaneously identifying the pendulum mass and
 * viscous friction coefficient.  The estimator models the pendulum
 * dynamics linearly with respect to these unknown parameters.  It
 * updates its estimates on each invocation using the current applied
 * torque, angular position, angular velocity and known physical
 * parameters.  A forgetting factor allows the estimator to track
 * slow changes such as added payloads or wear while gradually
 * discarding outdated information.  Basic tamper detection is
 * included by monitoring the residual between measured and predicted
 * torque.  If the residual remains consistently large a tamper
 * condition is flagged.
 *
 * The estimator is written in plain C to allow identical use on the
 * PC simulator and the embedded Pico firmware.  The API mirrors
 * the single‑parameter mass estimator but exposes both estimated
 * parameters and their tamper status.
 */

#ifndef ADAPTIVE_MASS_MASS_FRICTION_ESTIMATOR_H
#define ADAPTIVE_MASS_MASS_FRICTION_ESTIMATOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Structure holding the state of the mass and friction estimator.
 *
 * theta[0] holds the mass estimate (kg), theta[1] holds the
 * viscous friction coefficient estimate (N·m·s).  The 2×2 covariance
 * matrix P tracks the uncertainty of these estimates.  A forgetting
 * factor lambda tunes how quickly the estimator forgets old data.
 * The minimum/maximum bounds on mass and friction prevent the
 * estimates from wandering into unphysical regions.  The previous
 * angular velocity is stored for computing acceleration.  Moving
 * averages of the residual error support tamper detection.  A
 * tamper condition is raised when the normalised residual exceeds
 * a threshold for a sustained period.
 */
typedef struct {
    float theta[2];     /* [0]=mass estimate (kg), [1]=viscous friction (N·m·s) */
    float P[2][2];      /* 2×2 error covariance matrix */
    float lambda;       /* forgetting factor (0<lambda<=1) */
    float m_min, m_max; /* physical bounds on mass estimate */
    float b_min, b_max; /* physical bounds on friction estimate */
    float prev_omega;   /* previous angular velocity for acceleration */
    float error_avg;    /* exponential moving average of residual error */
    float error_var;    /* exponential moving average of squared residual error */
    int   tamper_count; /* counter for consecutive large residuals */
    bool  tampered;     /* flag indicating tamper condition */
    bool  initialized;  /* has the estimator been initialised? */
} mass_friction_estimator_t;

/*
 * Initialise the estimator with optional initial guesses for mass and
 * friction.  If either initial guess is non‑positive a conservative
 * default is used.  The covariance matrix is initialised to large
 * values to allow rapid adaptation.  Typical forgetting factors
 * around 0.98–0.995 provide good tracking of slow changes without
 * excessive noise.
 */
void mass_friction_estimator_init(mass_friction_estimator_t *est,
                                   float initial_mass,
                                   float initial_friction);

/*
 * Reset the estimator to an uninitialised state.  This clears the
 * tamper flag and residual history.  The next call to
 * mass_friction_estimator_update() will reinitialise the internal
 * state using the provided initial guesses from init().
 */
void mass_friction_estimator_reset(mass_friction_estimator_t *est);

/*
 * Perform a single RLS update of the estimator.  The inputs are the
 * applied torque (tau), upright‑referenced angle (theta), current
 * angular velocity (omega), timestep (dt), pendulum length (m) and
 * motor inertia (Jm).  The Coulomb friction magnitude (tau_ff) is
 * assumed known; its sign is applied internally based on omega.
 * Returns the updated mass estimate.  The friction estimate can be
 * obtained using mass_friction_estimator_get_friction().  This
 * function updates the tamper flag if a prolonged model mismatch is
 * observed.
 */
float mass_friction_estimator_update(mass_friction_estimator_t *est,
                                     float tau,
                                     float theta,
                                     float omega,
                                     float dt,
                                     float length,
                                     float Jm,
                                     float tau_ff);

/* Accessors for the current estimates.  These simply return the
 * current values stored in the estimator.  The caller can use the
 * returned mass and friction to update control parameters.  The
 * tamper predicate returns true if a tamper condition has been
 * detected.
 */
static inline float mass_friction_estimator_get_mass(const mass_friction_estimator_t *est) {
    return est->theta[0];
}
static inline float mass_friction_estimator_get_friction(const mass_friction_estimator_t *est) {
    return est->theta[1];
}
static inline bool mass_friction_estimator_is_tampered(const mass_friction_estimator_t *est) {
    return est->tampered;
}

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ADAPTIVE_MASS_MASS_FRICTION_ESTIMATOR_H */