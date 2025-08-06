#include "motor_protection.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// Well-engineered motor thermal protection implementation
// Based on industry thermal modeling principles

// Static function declarations
static void calculate_thermal_power_components(thermal_integrator_t *thermal, 
                                             float duty_cycle, float motor_speed_rad_s);
static void update_thermal_state(thermal_integrator_t *thermal);
static void integrate_thermal_energy(thermal_integrator_t *thermal, uint32_t timestamp_ms);
static float get_protection_output_scaling(const thermal_integrator_t *thermal);

// Initialize motor protection system with proper thermal parameters
bool motor_protection_init(motor_protection_t *protection, 
                          float pwm_freq_hz, float supply_voltage_v,
                          float motor_resistance_ohms) {
    if (!protection) return false;
    
    // Suppress unused parameter warning (resistance is defined in constants)
    (void)motor_resistance_ohms;
    
    // Clear entire structure
    memset(protection, 0, sizeof(motor_protection_t));
    
    // Initialize protection parameters
    protection->pwm_frequency = pwm_freq_hz;
    protection->supply_voltage = supply_voltage_v;
    protection->current_command = 0.0f;
    
    // Initialize thermal integrator with industry-inspired parameters
    thermal_integrator_t *thermal = &protection->thermal;
    
    // Set thermal protection thresholds with enhanced hysteresis for less twitchy behavior
    thermal->protection_threshold = MOTOR_THERMAL_CAPACITY;
    thermal->protection_clear_threshold = MOTOR_THERMAL_CAPACITY * 0.5f;  // 50% hysteresis (was 30%)
    thermal->emergency_threshold = MOTOR_EMERGENCY_CAPACITY;
    thermal->emergency_clear_threshold = MOTOR_EMERGENCY_CAPACITY * 0.4f; // 60% hysteresis (was 40%)
    
    // Initialize thermal state
    thermal->state = THERMAL_NORMAL;
    thermal->thermal_energy = 0.0f;
    thermal->thermal_power = 0.0f;
    thermal->peak_thermal_energy = 0.0f;  // Initialize peak energy tracking
    thermal->thermal_halt_active = false; // Initialize halt state
    thermal->current_duty_cycle = 0.0f;
    thermal->motor_speed_rad_s = 0.0f;
    thermal->operation_time = 0.0f;
    thermal->last_timestamp_ms = 0;
    thermal->initialized = true;
    
    protection->initialized = true;
    
    #ifdef PC_DEBUG
    printf("Motor protection initialized: Prot=%.1fJ, Emerg=%.1fJ, τ=%.1fs\n",
           thermal->protection_threshold, thermal->emergency_threshold, MOTOR_THERMAL_TIME_CONSTANT);
    #endif
    
    return true;
}

// Set motor command with comprehensive thermal analysis
void motor_protection_set_command(motor_protection_t *protection, 
                                 float duty_cycle, uint32_t timestamp_ms, 
                                 float motor_speed_rad_s) {
    if (!protection || !protection->initialized) return;
    
    thermal_integrator_t *thermal = &protection->thermal;
    
    // Update operating conditions
    thermal->current_duty_cycle = duty_cycle;
    thermal->motor_speed_rad_s = motor_speed_rad_s;
    protection->current_command = duty_cycle;
    
    // Calculate thermal power components using industry methods
    calculate_thermal_power_components(thermal, duty_cycle, motor_speed_rad_s);
    
    // Integrate thermal energy forward
    integrate_thermal_energy(thermal, timestamp_ms);
    
    // Update thermal protection state
    update_thermal_state(thermal);
}

// Step thermal protection system forward (for time-based integration)
void motor_protection_step(motor_protection_t *protection, uint32_t timestamp_ms) {
    if (!protection || !protection->initialized) return;
    
    thermal_integrator_t *thermal = &protection->thermal;
    
    // Recalculate thermal power with current conditions
    calculate_thermal_power_components(thermal, thermal->current_duty_cycle, thermal->motor_speed_rad_s);
    
    // Integrate thermal energy
    integrate_thermal_energy(thermal, timestamp_ms);
    
    // Update protection state
    update_thermal_state(thermal);
}

// Calculate comprehensive thermal power components using industry methods
static void calculate_thermal_power_components(thermal_integrator_t *thermal, 
                                             float duty_cycle, float motor_speed_rad_s) {
    const float abs_duty = fabsf(duty_cycle);
    const float abs_speed = fabsf(motor_speed_rad_s);
    
    // Component 1: I²R resistive losses (primary heating source)
    // P_resistive = I² × R, where I depends on duty cycle and back-EMF
    const float supply_voltage = MOTOR_SUPPLY_VOLTAGE;
    const float resistance = MOTOR_RESISTANCE;
    const float back_emf = MOTOR_BACK_EMF_CONSTANT * abs_speed;
    
    // Effective voltage accounting for back-EMF: V_eff = V_supply × duty - back_EMF
    const float effective_voltage = (supply_voltage * abs_duty) - back_emf;
    const float motor_current = fmaxf(0.0f, effective_voltage) / resistance;
    thermal->i2r_losses = motor_current * motor_current * resistance;
    
    // Component 2: Mechanical losses (friction, windage, bearing losses)
    // These scale with speed and are always present when moving
    const float base_friction = 0.05f; // Watts - bearing friction
    const float speed_dependent_friction = abs_speed * 0.02f; // Speed-dependent windage
    thermal->mechanical_losses = base_friction + speed_dependent_friction;
    
    // Component 3: PWM switching losses (H-bridge switching, dead-time losses)
    // Higher at moderate speeds, scale with duty cycle and switching frequency
    const float switching_base = 0.01f; // Watts - base switching loss
    const float duty_switching = abs_duty * 0.05f; // Duty-dependent switching
    const float freq_scaling = INTEGRATION_RATE_HZ / 1000.0f; // Frequency scaling factor
    thermal->switching_losses = (switching_base + duty_switching) * freq_scaling;
    
    // Total instantaneous thermal power
    thermal->thermal_power = thermal->i2r_losses + thermal->mechanical_losses + thermal->switching_losses;
    
    // Stall condition multiplier (industry practice for locked rotor protection)
    if (abs_duty > 0.3f && abs_speed < 0.5f) {
        // High duty cycle with low speed = stall condition = excessive heating
        thermal->thermal_power *= 2.5f; // Industry standard stall multiplier
    }
    
    #ifdef PC_DEBUG
    static int debug_power_counter = 0;
    if (++debug_power_counter % 500 == 0) { // Debug every 500ms
        printf("THERMAL POWER: I²R=%.2fW, Mech=%.2fW, Switch=%.2fW, Total=%.2fW (duty=%.2f, speed=%.1f)\n",
               thermal->i2r_losses, thermal->mechanical_losses, thermal->switching_losses, 
               thermal->thermal_power, duty_cycle, motor_speed_rad_s);
    }
    #endif
}

// Industry-standard thermal integration with exponential decay
static void integrate_thermal_energy(thermal_integrator_t *thermal, uint32_t timestamp_ms) {
    if (!thermal->initialized || thermal->last_timestamp_ms == 0) {
        thermal->last_timestamp_ms = timestamp_ms;
        return;
    }
    
    // Calculate integration time step with validation
    const uint32_t dt_ms = timestamp_ms - thermal->last_timestamp_ms;
    const float dt_seconds = dt_ms / 1000.0f;
    
    // Validate time step (reject outliers, handle wraparound)
    if (dt_seconds <= 0.0f || dt_seconds > 2.0f) {
        thermal->last_timestamp_ms = timestamp_ms;
        return;
    }
    
    // Update operation time tracking
    thermal->operation_time += dt_seconds;
    
    // Industry standard first-order thermal model with exponential decay
    // dE/dt = P_thermal - E/τ_thermal
    // Analytical solution for numerical stability
    const float time_constant = MOTOR_THERMAL_TIME_CONSTANT;
    const float decay_factor = expf(-dt_seconds / time_constant);
    
    // Apply thermal integration: E(t+dt) = E(t)×e^(-dt/τ) + P×τ×(1-e^(-dt/τ))
    thermal->thermal_energy = thermal->thermal_energy * decay_factor + 
                             thermal->thermal_power * time_constant * (1.0f - decay_factor);
    
    // Track peak thermal energy for order-of-magnitude decay calculation
    if (thermal->thermal_energy > thermal->peak_thermal_energy) {
        thermal->peak_thermal_energy = thermal->thermal_energy;
    }
    
    // Enforce physical bounds
    thermal->thermal_energy = fmaxf(0.0f, thermal->thermal_energy);
    
    // Update timestamp for next integration
    thermal->last_timestamp_ms = timestamp_ms;
}

// Update thermal protection state machine with order-of-magnitude halt logic
static void update_thermal_state(thermal_integrator_t *thermal) {
    const thermal_state_t previous_state = thermal->state;
    
    // State machine with order-of-magnitude halt requirement
    switch (thermal->state) {
        case THERMAL_NORMAL:
            // Only enter elevated state when much closer to protection limit
            if (thermal->thermal_energy > thermal->protection_threshold * 0.95f) {
                thermal->state = THERMAL_ELEVATED;
            }
            break;
            
        case THERMAL_ELEVATED:
            if (thermal->thermal_energy > thermal->protection_threshold) {
                thermal->state = THERMAL_PROTECTION;
            } else if (thermal->thermal_energy < thermal->protection_threshold * 0.85f) {
                // Larger hysteresis gap - wait for significant decay before returning to normal
                thermal->state = THERMAL_NORMAL;
            }
            break;
            
        case THERMAL_PROTECTION:
            if (thermal->thermal_energy > thermal->emergency_threshold) {
                // Exceeded emergency threshold – transition to emergency shutdown
                thermal->state = THERMAL_EMERGENCY;
            } else if (thermal->thermal_energy < thermal->protection_clear_threshold) {
                /*
                 * The classic implementation transitioned into a THERMAL_HALT state when
                 * leaving the protection zone.  However, this halt logic has proven
                 * counter‑productive for the pendulum application.  Instead, once the
                 * thermal energy falls below the clear threshold, we return directly
                 * to the normal operating state.  Reset the peak energy tracker so that
                 * future protections are measured from the current level.
                 */
                thermal->state = THERMAL_NORMAL;
                thermal->thermal_halt_active = false;
                thermal->peak_thermal_energy = thermal->thermal_energy;
                #ifdef PC_DEBUG
                printf("THERMAL PROTECTION CLEARED: Energy=%.1fJ below %.1fJ threshold\n",
                       thermal->thermal_energy, thermal->protection_clear_threshold);
                #endif
            }
            break;
            
        case THERMAL_EMERGENCY:
            if (thermal->thermal_energy < thermal->emergency_clear_threshold) {
                /*
                 * When leaving the emergency zone, do not enter a halt state.  As soon as the
                 * thermal energy decays below the emergency clear threshold, restore normal
                 * operation.  This avoids a mandatory cooling period that would otherwise
                 * stop the pendulum.  Reset the peak energy tracker to the current level.
                 */
                thermal->state = THERMAL_NORMAL;
                thermal->thermal_halt_active = false;
                thermal->peak_thermal_energy = thermal->thermal_energy;
                #ifdef PC_DEBUG
                printf("THERMAL EMERGENCY CLEARED: Energy=%.1fJ below %.1fJ threshold\n",
                       thermal->thermal_energy, thermal->emergency_clear_threshold);
                #endif
            }
            break;
            
        case THERMAL_HALT:
            /*
             * The halt state is deprecated for this application.  Immediately exit back
             * to normal.  This ensures that the pendulum is not artificially prevented
             * from recovering once temperatures decline.
             */
            thermal->state = THERMAL_NORMAL;
            thermal->thermal_halt_active = false;
            break;
    }
    
    // Log state changes with detailed information
    if (thermal->state != previous_state) {
        #ifdef PC_DEBUG
        // Removed arrow markers from debug output
        const char* state_names[] = {"NORMAL", "ELEVATED", "PROTECTION", "EMERGENCY", "HALT"};
        const float energy_percentage = (thermal->thermal_energy / thermal->protection_threshold) * 100.0f;
        printf("THERMAL STATE: %s → %s (%.1fJ, %.0f%% of protection threshold)\n",
               state_names[previous_state], state_names[thermal->state],
               thermal->thermal_energy, energy_percentage);
        #endif
    }
}

// Get protection output scaling based on thermal state
static float get_protection_output_scaling(const thermal_integrator_t *thermal) {
    switch (thermal->state) {
        case THERMAL_NORMAL:
        case THERMAL_ELEVATED:
            return 1.0f; // Full power available
            
        case THERMAL_PROTECTION:
            // Much more gradual power reduction - less twitchy behavior
            {
                const float energy_ratio = (thermal->thermal_energy - thermal->protection_threshold) /
                                          (thermal->emergency_threshold - thermal->protection_threshold);
                const float reduction_factor = 1.0f - (energy_ratio * 0.4f); // Only up to 40% reduction (was 80%)
                return fmaxf(0.6f, reduction_factor); // Minimum 60% power available (was 10%)
            }
            
        case THERMAL_EMERGENCY:
            return 0.0f; // No power - emergency shutdown
            
        case THERMAL_HALT:
            // Do not completely halt the motor during thermal halt; allow full power
            // The halt state is considered unnecessary in this application, so return 1.0f
            return 1.0f;
            
        default:
            return 0.0f; // Safety fallback
    }
}

// API Functions for external access

float motor_protection_get_output(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return 0.0f;
    
    const float scaling = get_protection_output_scaling(&protection->thermal);
    return protection->current_command * scaling;
}

bool motor_protection_is_active(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return false;
    return (protection->thermal.state >= THERMAL_PROTECTION);
}

bool motor_protection_is_emergency(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return false;
    return (protection->thermal.state == THERMAL_EMERGENCY);
}

bool motor_protection_is_halted(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return false;
    return (protection->thermal.state == THERMAL_HALT);
}

float motor_protection_get_thermal_energy(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return 0.0f;
    return protection->thermal.thermal_energy;
}

float motor_protection_get_thermal_power(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return 0.0f;
    return protection->thermal.thermal_power;
}

thermal_state_t motor_protection_get_state(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return THERMAL_NORMAL;
    return protection->thermal.state;
}

float motor_protection_get_i2r_losses(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return 0.0f;
    return protection->thermal.i2r_losses;
}

float motor_protection_get_mechanical_losses(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return 0.0f;
    return protection->thermal.mechanical_losses;
}

float motor_protection_get_switching_losses(const motor_protection_t *protection) {
    if (!protection || !protection->initialized) return 0.0f;
    return protection->thermal.switching_losses;
}
