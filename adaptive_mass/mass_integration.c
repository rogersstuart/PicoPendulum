/*
 * Adaptive Mass Integration Layer
 *
 * This implementation ties the low‑level mass estimator to the high‑level
 * pendulum controller.  It manages a single estimator instance and
 * performs the necessary housekeeping on each control cycle.  By
 * wrapping the estimator in this way the control logic in the
 * simulator and the embedded firmware remains identical.
 */

/*
 * Adaptive Mass Integration Layer
 *
 * This implementation binds the low‑level mass & friction estimator to
 * the high‑level pendulum controller.  It manages a single shared
 * estimator instance and coordinates the updates across PC and Pico
 * builds.  On each control cycle the integration layer computes the
 * applied torque from the motor command, updates the estimator with
 * the current angle and velocity, writes the updated mass and
 * friction estimates back into the control parameters and adjusts
 * the desired energy target.  If tampering is detected the control
 * state is forced into fault mode and the command is cleared.
 */

#include "adaptive_mass/mass_integration.h"
#include "adaptive_mass/mass_friction_estimator.h"
#include "embedded/config.h"

/* Local mass & friction estimator instance.  Each build (PC or Pico)
 * maintains its own estimator.  This variable must not be exposed
 * outside this module.
 */
static mass_friction_estimator_t g_shared_est;

/* Local helper to clamp a value between bounds.  Defined here to avoid
 * dragging in the clampf macro from control.c.  Using inline ensures
 * no overhead on embedded platforms.
 */
static inline float local_clamp(float x, float lo, float hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

void adaptive_mass_init(ctrl_params_t *p) {
    /* Initialise the shared estimator with the initial mass and friction
     * provided by the control parameters.  If p is NULL use defaults.
     */
    float initial_mass = 0.0f;
    float initial_fric = 0.0f;
    if (p) {
        initial_mass = p->m;
        initial_fric = p->b_vis;
    }
    mass_friction_estimator_init(&g_shared_est, initial_mass, initial_fric);
}

void adaptive_mass_reset(void) {
    mass_friction_estimator_reset(&g_shared_est);
}

void adaptive_mass_update(ctrl_params_t *p, ctrl_state_t *s, float u) {
    if (!p || !s) return;
    /* Lazy initialisation in case init() was not called. */
    if (!g_shared_est.initialized) {
        mass_friction_estimator_init(&g_shared_est, p->m, p->b_vis);
    }
    /* Clamp command to [-1,1] and convert to applied torque. */
    float cmd = local_clamp(u, -1.0f, 1.0f);
    float applied_tau = p->u_to_tau * cmd;
    /* Perform RLS update. */
    float new_mass = mass_friction_estimator_update(&g_shared_est,
                                                   applied_tau,
                                                   s->theta_u,
                                                   s->omega,
                                                   p->dt,
                                                   p->L,
                                                   p->Jm,
                                                   p->tau_ff);
    /* Update the control parameters with the new estimates. */
    p->m = new_mass;
    /* Also update viscous friction in the control parameters.  This
     * allows the controller to adapt its feedback gains or models if
     * friction drifts over time.  Friction is always non‑negative. */
    float new_fric = mass_friction_estimator_get_friction(&g_shared_est);
    p->b_vis = new_fric;
    /* Update the energy target for the new mass. */
    s->Edes = CALCULATE_ENERGY_TARGET(new_mass, p->L);
    /* If tampering has been detected force the fault state and zero the
     * command.  This protects the hardware and signals the user.
     */
    if (mass_friction_estimator_is_tampered(&g_shared_est)) {
        s->state = ST_FAULT;
        s->u = 0.0f;
    }
}