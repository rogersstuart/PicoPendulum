/*
 * Adaptive Mass Estimator Implementation
 *
 * This file implements a simple recursive least squares estimator for
 * identifying the pendulum mass in real time.  The estimator models
 * the pendulum dynamics linearly with respect to the unknown mass and
 * updates its estimate on each invocation using the current applied
 * torque, angular position, velocity and the system parameters.  A
 * forgetting factor allows the estimator to adapt to slow changes such
 * as added payloads while gradually discarding outdated information.
 *
 * Tamper detection is performed by monitoring the residual between the
 * measured torque and the model prediction.  Large sustained residuals
 * indicate either sensor faults or deliberate tampering (e.g. someone
 * grabbing the pendulum).  When detected the estimator asserts a
 * tampered flag which should be acted upon by the control layer.
 */

#include "adaptive_mass/mass_estimator.h"
#include <math.h>

/*
 * Default configuration constants.  These values were chosen based on
 * typical pendulum parameters and can be tuned if required.  The
 * forgetting factor close to unity allows the estimator to track slow
 * changes without being overly sensitive to noise.  The initial
 * covariance is set high to permit rapid adaptation on start-up.
 */
/* Larger initial covariance allows the estimator to adapt quickly even
 * when the initial mass guess is far from the true value.  A lower
 * forgetting factor causes the estimator to remember older data more
 * strongly, providing smoother estimates across a range of masses.
 */
#define MASS_ESTIMATOR_DEFAULT_P      50.0f
#define MASS_ESTIMATOR_DEFAULT_LAMBDA 0.99f
/* Bound the mass between 1 g and 1000 g (1 kg) as suggested.  Real
 * pendulum setups rarely exceed this range.  Keeping the maximum
 * reasonable prevents the estimator from drifting into unphysical
 * values during transients.
 */
#define MASS_ESTIMATOR_MIN_MASS       0.001f   /* 1 g */
#define MASS_ESTIMATOR_MAX_MASS       1.0f     /* 1 kg */

/* Tamper detection configuration.  The residual is normalised by the
 * predicted torque magnitude to yield a unitless measure.  If this
 * normalised error exceeds the defined threshold for more than
 * TAMPR_COUNT_THRESHOLD consecutive samples a tamper condition is
 * signalled.  The moving averages of the residual and its square
 * provide some smoothing to reduce the chance of false positives due
 * to noise.
 */
/* Increase error threshold and count threshold to reduce sensitivity to
 * model mismatch.  Normalised errors up to 150% of the predicted
 * torque are tolerated.  Sustained high errors over a longer period
 * trigger tamper protection.
 */
#define MASS_ESTIMATOR_ERROR_THRESHOLD        1.5f
#define MASS_ESTIMATOR_TAMPER_COUNT_THRESHOLD 200
#define MASS_ESTIMATOR_ERROR_ALPHA            0.995f

/* Skip updates during very energetic motion.  When the pendulum is
 * swinging rapidly the linearised model used by the estimator is
 * inaccurate; updates made in this regime would degrade the estimate.
 * These thresholds were chosen empirically to allow estimation when
 * velocities are modest but freeze it during violent swing-up.
 */
#define MASS_ESTIMATOR_OMEGA_THRESHOLD        8.0f   /* rad/s */
#define MASS_ESTIMATOR_ALPHA_THRESHOLD        50.0f  /* rad/s^2 */

/* Minimal covariance below which numerical precision issues can cause
 * divergence.  When P drops below this threshold it is reset to the
 * default value.  This implements covariance resetting suggested for
 * time-varying parameters.
 */
#define MASS_ESTIMATOR_MIN_COVARIANCE         1e-5f

/* Internal helper: sign function returning -1, 0 or +1 */
static inline float fsign(float x) {
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

void mass_estimator_init(mass_estimator_t *est, float initial_mass) {
    if (!est) return;
    /* If the provided initial mass is not sensible use the maximum
     * allowable mass as a starting point. This provides more aggressive
     * control initially, then adapts down as needed.
     */
    if (initial_mass <= 0.0f) {
        initial_mass = MASS_ESTIMATOR_MAX_MASS;
    }
    est->m_est = initial_mass;
    est->P = MASS_ESTIMATOR_DEFAULT_P;
    est->lambda = MASS_ESTIMATOR_DEFAULT_LAMBDA;
    est->m_min = MASS_ESTIMATOR_MIN_MASS;
    est->m_max = MASS_ESTIMATOR_MAX_MASS;
    est->prev_omega = 0.0f;
    est->error_avg = 0.0f;
    est->error_var = 0.0f;
    est->tamper_count = 0;
    est->tampered = false;
    est->initialized = true;
}

void mass_estimator_reset(mass_estimator_t *est) {
    if (!est) return;
    est->initialized = false;
    est->tampered = false;
    est->tamper_count = 0;
    est->error_avg = 0.0f;
    est->error_var = 0.0f;
    est->prev_omega = 0.0f;
}

float mass_estimator_update(mass_estimator_t *est,
                            float tau,
                            float theta,
                            float omega,
                            float dt,
                            float length,
                            float Jm,
                            float b_vis,
                            float tau_ff) {
    
    // Skip updates near the upright position where linearization is poor
    // The estimator works best when the pendulum is far from vertical
    if (fabsf(theta) < 0.3f) {  // Within ~17 degrees of upright
        // Don't update near the top - model is inaccurate here
        return est->m_hat;
    }
    
    // Skip updates when motor torque is too small to provide good signal
    // We need significant motor input for accurate estimation
    if (fabsf(tau) < 0.005f) {  // Less than 5mNm - too small
        return est->m_hat;
    }
    
    // CHANGED: Update primarily during high-torque phases of swing-up
    // Best estimation occurs when motor is actively driving the pendulum
    // This is typically when the pendulum is moving through the bottom region
    bool near_bottom = fabsf(fabsf(theta) - M_PI) < 1.0f;  // Within ~60 degrees of bottom
    bool significant_torque = fabsf(tau) > 0.01f;  // At least 10mNm
    bool moderate_velocity = fabsf(omega) > 0.5f && fabsf(omega) < 6.0f;  // Moving but not too fast
    
    // Only update when we have good conditions for estimation
    if (!near_bottom || !significant_torque || !moderate_velocity) {
        // Conditions not ideal for estimation
        return est->m_hat;
    }
    
    // Also skip if angular acceleration is too high (nonlinear effects dominate)
    float alpha = (tau - b_vis * omega - tau_ff * ctl_sgn(omega) + 
                   est->m_hat * GRAVITY_ACCEL * (length / 2.0f) * sinf(theta)) / 
                  (est->m_hat * length * length / 3.0f + Jm);
    
    if (fabsf(alpha) > 50.0f) {  // Acceleration too high - likely nonlinear
        return est->m_hat;
    }
    /* Lazy initialise if the estimator was reset. */
    if (!est->initialized) {
        mass_estimator_init(est, est->m_est > 0.0f ? est->m_est : 0.1f);
        est->prev_omega = omega;
    }
    /* Protect against tiny timesteps. */
    if (dt <= 0.0f) {
        return est->m_est;
    }
    /* Compute angular acceleration using backward difference. */
    alpha = (omega - est->prev_omega) / dt;
    est->prev_omega = omega;

    /* Skip estimator update when motion is too aggressive.  During
     * high‑velocity or high‑acceleration phases the simple linear model
     * underpinning the estimator breaks down.  Instead of performing an
     * update we simply update the residual statistics for tamper
     * detection and return the current mass.  This keeps the estimate
     * stable while still monitoring for tampering.
     */
    if (fabsf(omega) > MASS_ESTIMATOR_OMEGA_THRESHOLD || fabsf(alpha) > MASS_ESTIMATOR_ALPHA_THRESHOLD) {
        /* Compute regressor as usual for residual tracking. */
        const float g = 9.81f;
        float phi_skip = (length * length / 3.0f) * alpha - (g * length / 2.0f) * sinf(theta);
        float sgn_omega_skip = fsign(omega);
        float y_skip = tau - (Jm * alpha) - (b_vis * omega) - (tau_ff * sgn_omega_skip);
        float err_skip = y_skip - phi_skip * est->m_est;
        est->error_avg = MASS_ESTIMATOR_ERROR_ALPHA * est->error_avg + (1.0f - MASS_ESTIMATOR_ERROR_ALPHA) * err_skip;
        est->error_var = MASS_ESTIMATOR_ERROR_ALPHA * est->error_var + (1.0f - MASS_ESTIMATOR_ERROR_ALPHA) * err_skip * err_skip;
        /* Increment tamper counter if error is high relative to predicted torque. */
        float predicted_skip = (Jm * alpha) + (est->m_est * phi_skip) + (b_vis * omega) + (tau_ff * sgn_omega_skip);
        float norm_skip = fabsf(predicted_skip);
        if (norm_skip < 1e-3f) norm_skip = 1e-3f;
        float normalised_error_skip = fabsf(err_skip) / norm_skip;
        if (normalised_error_skip > MASS_ESTIMATOR_ERROR_THRESHOLD) {
            est->tamper_count++;
            if (est->tamper_count > MASS_ESTIMATOR_TAMPER_COUNT_THRESHOLD) {
                est->tampered = true;
            }
        } else {
            est->tamper_count = 0;
        }
        return est->m_est;
    }
    /* Compute regressor for mass.  Using an upright-referenced angle the
     * gravitational torque term is negative.  Starting from the dynamic
     * equation:
     *    tau = (m*L^2/3 + Jm)*alpha + m*g*(L/2)*sin(theta_b) + friction,
     * where theta_b = theta_u + π.  Substituting sin(theta_b) = -sin(theta)
     * yields:
     *    tau = Jm*alpha + m*(L^2/3*alpha - g*L/2*sin(theta)) + friction.
     * Therefore the regressor for mass is:
     *    phi = (L^2/3)*alpha - g*(L/2)*sin(theta)
     */
    const float g = 9.81f;
    float phi = (length * length / 3.0f) * alpha - (g * length / 2.0f) * sinf(theta);
    /* Compute measured output with known terms removed: y = tau - Jm*alpha - b_vis*omega - tau_ff*sgn(omega) */
    float sgn_omega = fsign(omega);
    float y = tau - (Jm * alpha) - (b_vis * omega) - (tau_ff * sgn_omega);
    /* Prevent degenerate regressor; if phi is extremely small the update would be ill-conditioned. */
    if (fabsf(phi) < 1e-6f) {
        /* Still update tamper detection on zero regressor because error
         * might accumulate. */
        float err = y - phi * est->m_est;
        /* Exponential moving averages for residual. */
        est->error_avg = MASS_ESTIMATOR_ERROR_ALPHA * est->error_avg + (1.0f - MASS_ESTIMATOR_ERROR_ALPHA) * err;
        est->error_var = MASS_ESTIMATOR_ERROR_ALPHA * est->error_var + (1.0f - MASS_ESTIMATOR_ERROR_ALPHA) * err * err;
        /* No parameter update; return current estimate. */
        return est->m_est;
    }
    /* Compute Kalman gain. */
    float P_phi = est->P * phi;
    float denom = est->lambda + phi * P_phi;
    float K = P_phi / denom;
    /* Innovation / residual. */
    float err = y - phi * est->m_est;
    /* Update mass estimate. */
    est->m_est += K * err;
    /* Enforce physical bounds. */
    if (est->m_est < est->m_min) est->m_est = est->m_min;
    if (est->m_est > est->m_max) est->m_est = est->m_max;
    /* Update covariance. */
    est->P = (1.0f / est->lambda) * (est->P - K * phi * est->P);

    /* Covariance resetting: if P becomes extremely small it will slow
     * adaptation and can introduce numerical issues.  Reset it to the
     * default value suggested by industrial practice for time‑varying
     * parameters.
     */
    if (est->P < MASS_ESTIMATOR_MIN_COVARIANCE) {
        est->P = MASS_ESTIMATOR_DEFAULT_P;
    }
    /* Update residual statistics for tamper detection. */
    est->error_avg = MASS_ESTIMATOR_ERROR_ALPHA * est->error_avg + (1.0f - MASS_ESTIMATOR_ERROR_ALPHA) * err;
    est->error_var = MASS_ESTIMATOR_ERROR_ALPHA * est->error_var + (1.0f - MASS_ESTIMATOR_ERROR_ALPHA) * err * err;
    /* Normalise error by predicted magnitude to form a relative metric.  The
     * predicted torque reconstructs the full dynamic model using the
     * estimated mass and the known terms.
     */
    float predicted = (Jm * alpha) + (est->m_est * phi) + (b_vis * omega) + (tau_ff * sgn_omega);
    float norm = fabsf(predicted);
    if (norm < 1e-3f) norm = 1e-3f;
    float normalised_error = fabsf(err) / norm;
    /* Tamper logic: increment counter when error is persistently large. */
    if (normalised_error > MASS_ESTIMATOR_ERROR_THRESHOLD) {
        est->tamper_count++;
        if (est->tamper_count > MASS_ESTIMATOR_TAMPER_COUNT_THRESHOLD) {
            est->tampered = true;
        }
    } else {
        /* Reset counter on acceptable errors. */
        est->tamper_count = 0;
    }
    return est->m_est;
}