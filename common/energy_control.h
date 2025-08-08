/*
 * Energy Control Module
 *
 * This header declares a standalone energy control function used by
 * both the embedded controller and the PC simulator.  Factoring
 * swing‑up logic into its own module reduces duplication between
 * platform builds and makes the code easier to maintain.  The
 * implementation can be found in energy_control.c.
 */

#ifndef COMMON_ENERGY_CONTROL_H
#define COMMON_ENERGY_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "embedded/control.h"

/* Compute the energy pump command during swing‑up.  Given the control
 * parameters and current state, this function returns the duty cycle
 * to apply to the motor in swing‑up mode.  It updates internal state
 * variables within the ctrl_state_t (kick flags, drive level,
 * continuous rotations) and uses the unified virtual encoder to
 * detect spin‑out conditions.  Saturation of the returned command
 * should be performed by the caller if necessary.
 */
float energy_control(const ctrl_params_t *p, ctrl_state_t *s);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* COMMON_ENERGY_CONTROL_H */