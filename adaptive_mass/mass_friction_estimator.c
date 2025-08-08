/*
 * Adaptive Mass & Friction Estimator Implementation
 *
 * This file implements a two‑parameter recursive least squares (RLS)
 * estimator.  It identifies both the pendulum mass and the viscous
 * friction coefficient in real time.  The estimator assumes that
 * Coulomb friction (static friction) is provided externally via the
 * tau_ff parameter.  The underlying model of the pendulum dynamics is
 * linear with respect to the unknown parameters:
 *
 *     tau = Jm * alpha
 *           + m * [ (L^2/3) * alpha - g*(L/2)*sin(theta) ]
 *           + b_vis * omega
 *           + tau_ff * sgn(omega)
 *
 * where:
 *   - tau is the applied torque from the motor (N·m), positive in
 *     the direction that would increase theta.
 *   - alpha is the angular acceleration (rad/s²).
 *   - m is the pendulum mass (kg).
 *   - b_vis is the viscous friction coefficient (N·m·s).
 *   - Jm is the motor/hub inertia about the axis (kg·m²).
 *   - L is the pendulum length (m).
 *   - g is the gravitational acceleration (9.81 m/s²).
 *   - theta is the upright‑referenced angle (rad), i.e. 0 at the
 *     inverted equilibrium, positive to the right.
 *   - omega is the angular velocity (rad/s).
 *
 * The regressor vector for the unknown parameters [m, b_vis] is
 * therefore phi = [ (L^2/3)*alpha - g*(L/2)*sin(theta), omega ].
 * The measured output with known terms removed is y = tau - Jm*alpha
 * - tau_ff*sgn(omega).  With these definitions the estimator solves
 * y ≈ phi·theta.
 *
 * A standard RLS update with forgetting factor lambda is used.  The
 * covariance matrix is reset to large values if it becomes too
 * small.  Residual statistics drive tamper detection: when the
 * normalised residual remains above a threshold for a sustained
 * period, tamper_flag becomes true.
 */

#include "adaptive_mass/mass_friction_estimator.h"
#include <math.h>

/* Default configuration.  The initial covariance values reflect
 * uncertainty in both mass and friction.  Mass is typically in the
 * range [0.05, 1] kg, friction in [0, 0.1] N·m·s.  The diagonal
 * elements of P are initialised accordingly.  The off‑diagonal terms
 * start at zero, meaning the parameters are assumed independent.  A
 * forgetting factor of 0.985 provides good balance between tracking
 * slow variations and noise rejection.  Bounds prevent the estimates
 * from drifting into unphysical values.  Tamper detection uses a
 * normalised residual threshold of 2.0 (i.e. residuals twice as
 * large as the predicted torque) over 500 consecutive samples.
 */
#define MF_EST_DEFAULT_P_MASS      100.0f
#define MF_EST_DEFAULT_P_FRICT     0.01f
#define MF_EST_DEFAULT_LAMBDA      0.985f
#define MF_EST_MIN_MASS            0.05f   /* 50 g */
#define MF_EST_MAX_MASS            1.5f    /* 1.5 kg */
#define MF_EST_MIN_FRICT           0.0f
#define MF_EST_MAX_FRICT           0.3f
#define MF_EST_ERROR_THRESHOLD     2.0f
#define MF_EST_TAMPER_COUNT_MAX    500
#define MF_EST_ERROR_ALPHA         0.995f
#define MF_EST_OMEGA_THRESH        10.0f   /* rad/s */
#define MF_EST_ALPHA_THRESH        80.0f   /* rad/s² */
#define MF_EST_MIN_COVARIANCE      1e-6f

/* Internal helper: sign function returning -1, 0 or +1. */
static inline float sign_f(float x) {
    return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

void mass_friction_estimator_init(mass_friction_estimator_t *est,
                                   float initial_mass,
                                   float initial_friction) {
    if (!est) return;
    /* Use conservative defaults if the provided initial guesses are
     * invalid.  Starting with maximum mass provides more aggressive
     * control initially, then adapts down as needed. */
    if (initial_mass <= 0.0f) {
        initial_mass = MF_EST_MAX_MASS;
    }
    if (initial_friction < 0.0f) {
        initial_friction = 0.5f * (MF_EST_MIN_FRICT + MF_EST_MAX_FRICT);
    }
    est->theta[0] = initial_mass;
    est->theta[1] = initial_friction;
    /* Initialise covariance matrix. */
    est->P[0][0] = MF_EST_DEFAULT_P_MASS;
    est->P[1][1] = MF_EST_DEFAULT_P_FRICT;
    est->P[0][1] = 0.0f;
    est->P[1][0] = 0.0f;
    est->lambda   = MF_EST_DEFAULT_LAMBDA;
    est->m_min    = MF_EST_MIN_MASS;
    est->m_max    = MF_EST_MAX_MASS;
    est->b_min    = MF_EST_MIN_FRICT;
    est->b_max    = MF_EST_MAX_FRICT;
    est->prev_omega = 0.0f;
    est->error_avg = 0.0f;
    est->error_var = 0.0f;
    est->tamper_count = 0;
    est->tampered = false;
    est->initialized = true;
}

void mass_friction_estimator_reset(mass_friction_estimator_t *est) {
    if (!est) return;
    est->initialized   = false;
    est->tampered      = false;
    est->tamper_count  = 0;
    est->error_avg     = 0.0f;
    est->error_var     = 0.0f;
    est->prev_omega    = 0.0f;
}

float mass_friction_estimator_update(mass_friction_estimator_t *est,
                                      float tau,
                                      float theta,
                                      float omega,
                                      float dt,
                                      float length,
                                      float Jm,
                                      float tau_ff) {
    
    // Skip updates near the upright position where the model is poor
    if (fabsf(theta) < 0.3f) {  // Within ~17 degrees of upright
        // Don't update near the top - linearization fails here
        return est->theta[0];
    }
    
    // Skip updates when motor torque is insignificant
    if (fabsf(tau) < 0.005f) {  // Less than 5mNm
        return est->theta[0];
    }
    
    // CHANGED: Focus estimation on high-torque regions near bottom
    // The pendulum model is most accurate when:
    // 1. We're away from singularities (not at top)
    // 2. Motor is applying significant torque
    // 3. Velocity is moderate (not stalled, not too fast)
    
    // Check if we're in a good region for estimation
    bool near_bottom = fabsf(fabsf(theta) - M_PI) < 1.2f;  // Within ~70 degrees of bottom
    bool good_torque = fabsf(tau) > 0.015f;  // At least 15mNm for friction estimation
    bool good_velocity = fabsf(omega) > 0.3f && fabsf(omega) < 8.0f;
    
    // Additional check: motor should be actively driving (not just braking)
    bool motor_driving = (tau * omega > 0);  // Same sign = adding energy
    
    if (!near_bottom || !good_torque || !good_velocity) {
        // Not ideal conditions for joint mass/friction estimation
        return est->theta[0];
    }
    
    // For friction estimation, we particularly want phases where motor is driving
    // against friction (not coasting)
    if (!motor_driving && fabsf(tau) < 0.02f) {
        // Coasting or light braking - poor for friction estimation
        return est->theta[0];
    }
    if (!est->initialized) {
        mass_friction_estimator_init(est, est->theta[0] > 0.0f ? est->theta[0] : 0.1f,
                                     est->theta[1] >= 0.0f ? est->theta[1] : 0.01f);
        est->prev_omega = omega;
    }
    if (dt <= 0.0f) {
        return est->theta[0];
    }
    /* Compute angular acceleration. */
    float alpha = (omega - est->prev_omega) / dt;
    est->prev_omega = omega;
    /* If motion is extremely aggressive, skip update to avoid
     * corrupting the estimate.  Still update residual statistics. */
    if (fabsf(omega) > MF_EST_OMEGA_THRESH || fabsf(alpha) > MF_EST_ALPHA_THRESH) {
        /* Compute regressor and residual for tamper tracking. */
        const float g = 9.81f;
        float phi_mass = (length * length / 3.0f) * alpha - (g * length / 2.0f) * sinf(theta);
        float phi_fric = omega;
        float sgn_omega = sign_f(omega);
        float y = tau - (Jm * alpha) - (tau_ff * sgn_omega);
        float pred = est->theta[0] * phi_mass + est->theta[1] * phi_fric;
        float err = y - pred;
        est->error_avg = MF_EST_ERROR_ALPHA * est->error_avg + (1.0f - MF_EST_ERROR_ALPHA) * err;
        est->error_var = MF_EST_ERROR_ALPHA * est->error_var + (1.0f - MF_EST_ERROR_ALPHA) * err * err;
        float norm = fabsf(pred);
        if (norm < 1e-3f) norm = 1e-3f;
        float nerr = fabsf(err) / norm;
        if (nerr > MF_EST_ERROR_THRESHOLD) {
            est->tamper_count++;
            if (est->tamper_count > MF_EST_TAMPER_COUNT_MAX) {
                est->tampered = true;
            }
        } else {
            est->tamper_count = 0;
        }
        return est->theta[0];
    }
    /* Compute regressor vector phi. */
    const float g = 9.81f;
    float phi[2];
    phi[0] = (length * length / 3.0f) * alpha - (g * length / 2.0f) * sinf(theta);
    phi[1] = omega;
    /* Compute measured output with known terms removed. */
    float sgn_omega = sign_f(omega);
    float y = tau - (Jm * alpha) - (tau_ff * sgn_omega);
    /* Compute P*phi vector and phi^T P phi scalar. */
    float P_phi[2];
    P_phi[0] = est->P[0][0] * phi[0] + est->P[0][1] * phi[1];
    P_phi[1] = est->P[1][0] * phi[0] + est->P[1][1] * phi[1];
    float phi_P_phi = phi[0] * P_phi[0] + phi[1] * P_phi[1];
    float denom = est->lambda + phi_P_phi;
    if (denom < 1e-6f) denom = 1e-6f;
    /* Compute Kalman gain K = (P*phi) / denom. */
    float K[2];
    K[0] = P_phi[0] / denom;
    K[1] = P_phi[1] / denom;
    /* Compute residual (innovation). */
    float pred = est->theta[0] * phi[0] + est->theta[1] * phi[1];
    float err = y - pred;
    /* Update parameter estimates. */
    est->theta[0] += K[0] * err;
    est->theta[1] += K[1] * err;
    /* Enforce bounds on mass and friction. */
    if (est->theta[0] < est->m_min) est->theta[0] = est->m_min;
    if (est->theta[0] > est->m_max) est->theta[0] = est->m_max;
    if (est->theta[1] < est->b_min) est->theta[1] = est->b_min;
    if (est->theta[1] > est->b_max) est->theta[1] = est->b_max;
    /* Update covariance: P = (1/lambda) * (P - K*phi^T*P).  Because
     * phi is a column vector and K is computed from P*phi, we can
     * express the update elementwise. */
    float P_tmp[2][2];
    /* Compute K*phi^T*P = K * (phi^T * P).  First compute phi^T * P. */
    float phi_T_P[2];
    phi_T_P[0] = phi[0] * est->P[0][0] + phi[1] * est->P[1][0];
    phi_T_P[1] = phi[0] * est->P[0][1] + phi[1] * est->P[1][1];
    /* Compute outer product K * (phi^T * P). */
    P_tmp[0][0] = K[0] * phi_T_P[0];
    P_tmp[0][1] = K[0] * phi_T_P[1];
    P_tmp[1][0] = K[1] * phi_T_P[0];
    P_tmp[1][1] = K[1] * phi_T_P[1];
    /* Update P. */
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            est->P[i][j] = (est->P[i][j] - P_tmp[i][j]) / est->lambda;
        }
    }
    /* Covariance reset if it becomes too small. */
    if (est->P[0][0] < MF_EST_MIN_COVARIANCE || est->P[1][1] < MF_EST_MIN_COVARIANCE) {
        est->P[0][0] = MF_EST_DEFAULT_P_MASS;
        est->P[1][1] = MF_EST_DEFAULT_P_FRICT;
        est->P[0][1] = 0.0f;
        est->P[1][0] = 0.0f;
    }
    /* Update residual statistics for tamper detection. */
    est->error_avg = MF_EST_ERROR_ALPHA * est->error_avg + (1.0f - MF_EST_ERROR_ALPHA) * err;
    est->error_var = MF_EST_ERROR_ALPHA * est->error_var + (1.0f - MF_EST_ERROR_ALPHA) * err * err;
    /* Normalise the residual by predicted torque magnitude. */
    float norm = fabsf(pred);
    if (norm < 1e-3f) norm = 1e-3f;
    float normalised_err = fabsf(err) / norm;
    if (normalised_err > MF_EST_ERROR_THRESHOLD) {
        est->tamper_count++;
        if (est->tamper_count > MF_EST_TAMPER_COUNT_MAX) {
            est->tampered = true;
        }
    } else {
        est->tamper_count = 0;
    }
    return est->theta[0];
}