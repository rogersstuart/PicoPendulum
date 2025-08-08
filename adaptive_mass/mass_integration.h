/*
 * Adaptive Mass Integration Layer
 *
 * This header declares a set of helper functions that integrate the
 * generic mass estimator into the pendulum control algorithm.  It wraps
 * the low‑level mass_estimator routines and manages a shared estimator
 * instance.  By calling these functions from both the embedded and
 * simulator builds, we ensure that mass estimation behaves identically
 * across platforms.
 */

#ifndef ADAPTIVE_MASS_MASS_INTEGRATION_H
#define ADAPTIVE_MASS_MASS_INTEGRATION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declare control structures without including control.h here to
 * avoid circular dependencies when this header is included from C++.
 * The definitions are provided by embedded/control.h or an equivalent
 * header in the simulator.  Including control.h directly here is
 * avoided because the integration layer must remain agnostic to the
 * specific platform. */
#include "embedded/control.h"

/* Initialise the adaptive estimator using the initial mass and
 * friction contained in the provided control parameters.  Passing
 * NULL causes the estimator to use built‑in defaults.  This
 * function should be called once during controller initialisation.
 */
void adaptive_mass_init(ctrl_params_t *p);

/* Reset the estimator back to an uninitialised state.  Use when the
 * pendulum system is changed or after a tamper event.  The next
 * adaptive_mass_update() call will reinitialise the estimator
 * automatically using the most recently provided initial values.
 */
void adaptive_mass_reset(void);

/* Perform one adaptive update using the current control command and
 * state.  The motor command `u` must reflect the final duty cycle
 * applied to the motor after saturations and inversions.  The
 * estimator uses this to compute the applied torque and then
 * reconstruct the dynamic model to update the mass and friction
 * estimates.  Updated parameters are written back into the control
 * structure and the desired energy target is recomputed.  If the
 * residuals persistently exceed a threshold the control state is
 * forced into ST_FAULT and the command is cleared.  Call exactly
 * once per control cycle after computing the motor command but before
 * applying it.
 */
void adaptive_mass_update(ctrl_params_t *p, ctrl_state_t *s, float u);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ADAPTIVE_MASS_MASS_INTEGRATION_H */