#ifndef PWM_SIMULATION_H
#define PWM_SIMULATION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// PWM signal generator - simulates real hardware PWM
typedef struct {
    float frequency_hz;      // PWM frequency
    float duty_cycle;        // Current duty cycle [0.0, 1.0]
    float phase_offset;      // Phase offset for complementary signals
    double time_accumulator; // Internal time tracking for waveform generation
    bool pin1_state;         // Current state of pin 1
    bool pin2_state;         // Current state of pin 2
    float supply_voltage;    // Supply voltage (e.g., 5V)
} pwm_generator_t;

// Motor model that responds to real PWM waveform
typedef struct {
    float resistance;        // Motor winding resistance (ohms)
    float inductance;        // Motor winding inductance (henries)
    float back_emf_constant; // Back EMF constant (V⋅s/rad)
    float torque_constant;   // Torque constant (N⋅m/A)
    float rotor_inertia;     // Rotor inertia (kg⋅m²)
    float viscous_damping;   // Viscous damping coefficient
    
    // Dynamic state
    float current;           // Motor current (amps)
    float angular_velocity;  // Motor angular velocity (rad/s)
    float applied_voltage;   // Instantaneous applied voltage
    float back_emf;          // Back EMF voltage
    float torque_output;     // Output torque
    
    // PWM sampling
    float voltage_samples[1000]; // Circular buffer for voltage samples
    int sample_index;            // Current sample index
    int samples_per_pwm_cycle;   // Number of samples per PWM cycle
} motor_model_t;

// Initialize PWM generator
void pwm_generator_init(pwm_generator_t *pwm, float frequency_hz, float supply_voltage);

// Set PWM duty cycle for sign-magnitude mode
void pwm_generator_set_sign_magnitude(pwm_generator_t *pwm, float command);

// Set PWM duty cycle for locked anti-phase mode
void pwm_generator_set_locked_antiphase(pwm_generator_t *pwm, float command);

// Step PWM generator forward in time and return instantaneous voltage
float pwm_generator_step(pwm_generator_t *pwm, float dt_seconds);

// Initialize motor model
void motor_model_init(motor_model_t *motor, float resistance, float inductance,
                      float back_emf_k, float torque_k, float inertia, float damping,
                      float pwm_frequency, float simulation_frequency);

// Apply instantaneous voltage to motor and step physics
void motor_model_apply_voltage(motor_model_t *motor, float voltage, float dt_seconds);

// Get motor torque output
float motor_model_get_torque(const motor_model_t *motor);

// Get motor current (for protection system)
float motor_model_get_current(const motor_model_t *motor);

// Get RMS current over last PWM cycle (for thermal calculation)
float motor_model_get_rms_current(const motor_model_t *motor);

// Set motor angular velocity (from pendulum physics)
void motor_model_set_angular_velocity(motor_model_t *motor, float omega);

#ifdef __cplusplus
}
#endif

#endif // PWM_SIMULATION_H
