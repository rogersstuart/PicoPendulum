/*
 * Pico Pendulum main
 * - I2C AS5600 angle sensor
 * - DRV8833 H-bridge (GP14, GP15) 1kHz PWM (optimized for static friction breakaway)
 * - 1 kHz control loop
 * - 2-state Kalman filter for [theta, omega]
 * - UART/USB CSV logging
 *
 * CSV columns:
 * t, state, theta_b, theta_u, omega, u_cmd, raw_angle, agc, mag, i2c_ok
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "hardware/watchdog.h"  // Add this for watchdog functions
#include "as5600.h"
#include "drv8833.h"
#include "motor_protection.h"  // For motor_protection_step
#include "filters.h"
#include "control.h"
#include "debug.h"

#define LED_PIN 25
#define I2C_SDA 2
#define I2C_SCL 3
#define PWM_IN1 14
#define PWM_IN2 15
#define CONTROL_HZ 1000.0f

// Optional: set to true to invert motor polarity if needed
bool MOTOR_INVERT = false;

// Optional: set to true to invert encoder direction if needed
bool SENSOR_INVERT = true;

// Globals
as5600_t enc;              // Remove static - needed by debug.c
drv8833_t drv;             // Remove static - needed by debug.c
ctrl_params_t P;           // Remove static - needed by debug.c
ctrl_state_t  S;           // Remove static - needed by debug.c
static KalmanFilter KF;
lpf1_t lp_angle;           // Remove static - needed by debug.c
diff_lpf_t dtheta;         // Remove static - needed by debug.c

// Add missing globals
volatile bool control_enabled = false;  // Remove static - needed by debug.c
static volatile bool in_cb = false;
volatile bool enc_found = false;                  // Remove static - needed by debug.c
bool thermal_protection_active = false; // Remove static - needed by debug.c
static float tsec = 0.0f;
static float theta_cal = 0.0f;

// helpers
static inline float wrap_pi(float x) {
    while (x > M_PI)  x -= 2*M_PI;
    while (x <= -M_PI) x += 2*M_PI;
    return x;
}

static inline float wrap_2pi(float x) {
    while (x >= 2*M_PI) x -= 2*M_PI;
    while (x < 0)       x += 2*M_PI;
    return x;
}

static volatile bool loop_overrun = false;
static volatile uint32_t miss_i2c = 0;

bool repeating_timer_cb(struct repeating_timer *t) {
    static uint32_t overrun_count = 0;
    static uint32_t cb_count = 0;
    
    // Immediate return if not enabled
    if (!control_enabled) return true;
    
    // Check for callback overrun
    if (in_cb) {
        overrun_count++;
        // Only print if debug output is enabled
        if (overrun_count % 100 == 0 && debug_is_output_enabled()) {
            printf("WARNING: Timer overrun count: %lu\n", overrun_count);
        }
        return true;
    }
    
    in_cb = true;
    absolute_time_t start = get_absolute_time();
    
    // Safely read encoder with timeout
    float theta_raw = 0.0f;
    bool encoder_ok = false;
    
    if (enc_found) {
        // Try to read angle in radians directly
        encoder_ok = as5600_read_angle_rad(&enc, &theta_raw);
        
        if (!encoder_ok) {
            static uint32_t error_count = 0;
            error_count++;
            // Only print error message every 1000 failures if debug is enabled
            if (error_count % 1000 == 0 && debug_is_output_enabled()) {
                printf("Encoder read failed! (count: %lu)\n", error_count);
            }
            if (error_count > 5000) {
                enc_found = false;  // Mark encoder as failed after many attempts
            }
        }
    }
    
    if (!encoder_ok) {
        // Use fallback or safe mode
        in_cb = false;
        return true;
    }
    
    // Unwrap and compute bottom/upright frames
    static float angle_prev = 0.0f;
    float angle_unwrapped = unwrap_angle(angle_prev, theta_raw);
    angle_prev = angle_unwrapped;

    // Apply sensor inversion if needed (to fix encoder direction mismatch)
    if (SENSOR_INVERT) {
        angle_unwrapped = -angle_unwrapped;
    }

    // **CORRECTED ANGLE REFERENCES**
    // Calculate theta_b as angle from bottom: theta_b=0 when hanging down, theta_b=π when upright
    float theta_b = wrap_pi(angle_unwrapped - enc.angle_offset_rad);
    float theta_u = wrap_pi(theta_b - (float)M_PI); // upright = 0

    // 2) Estimate omega with dynamic dt calculation
    static absolute_time_t last_time;
    static bool first_iteration = true;
    
    float dt;
    if (first_iteration) {
        dt = 1.0f / CONTROL_HZ;  // Use nominal dt for first iteration
        last_time = start;
        first_iteration = false;
    } else {
        // Calculate actual elapsed time for more accurate derivative
        int64_t elapsed_us = absolute_time_diff_us(last_time, start);
        dt = elapsed_us / 1000000.0f;  // Convert to seconds
        
        // Sanity check: clamp dt to reasonable bounds
        if (dt < 0.0005f || dt > 0.005f) {  // 0.5ms to 5ms
            dt = 1.0f / CONTROL_HZ;  // Fall back to nominal if too far off
        }
        last_time = start;
    }
    
    float theta_meas = lpf1_step(&lp_angle, theta_u);
    float omega_meas = diff_lpf_step(&dtheta, theta_meas, dt);
    
    // Calculate RAW velocity for energy control (swing-up)
    static float theta_u_prev = 0.0f;
    static bool theta_prev_initialized = false;
    float omega_raw = 0.0f;
    if (theta_prev_initialized) {
        omega_raw = (theta_u - theta_u_prev) / dt;
    } else {
        omega_raw = 0.0f;
        theta_prev_initialized = true;
    }
    theta_u_prev = theta_u;

    // Kalman predict/update
    // CRITICAL FIX: DO NOT re-initialize the Kalman filter here.
    // It must only be initialized once at startup.
    // kalman_init(&KF, dt, a, b, 1e-4f, 5e-3f, 3e-3f); // <-- DELETE OR COMMENT OUT THIS LINE
    
    // Use the previous control command for prediction (S.u from last iteration)
    kalman_predict(&KF, S.u);
    kalman_update(&KF, theta_meas);

    S.theta_b = theta_b;
    // Store LIGHTLY FILTERED values for energy control (swing-up) and heavily filtered for balance
    S.theta_u = theta_meas;  // Lightly filtered angle for energy control (helps with noise)
    S.omega   = omega_meas;  // Lightly filtered velocity for energy control (helps with noise)
    
    // Store Kalman-filtered values for balance controller use
    S.theta_u_filtered = kalman_get_theta(&KF);
    S.omega_filtered   = kalman_get_omega(&KF);

    // 3) Update control state machine
    // Call the main step function, which now correctly wraps ctrl_update.
    float u = ctrl_step(&P, &S);
    
    // Check if debug system has motor control override
    float debug_motor_value = 0.0f;
    if (debug_has_motor_control(&debug_motor_value)) {
        // Debug system has control - use debug command (already handles inversion if needed)
        u = debug_motor_value;
        
        // If emergency stop, ensure control loop knows to go to IDLE
        debug_motor_state_t* motor_state = debug_get_motor_state();
        if (motor_state->emergency_stop) {
            S.state = ST_IDLE;
            S.u = 0.0f;
        }
    }
    
    // FIXED: Motor inversion is now handled inside the energy controller
    // This ensures consistent sign convention for energy pumping
    // The control command u already has the correct sign for the motor
    
    // Update the state with the final command for Kalman prediction
    S.u = u;
    
    // CRITICAL: Always update thermal protection system, even when motor command is zero
    // This ensures thermal energy can decay when the system is idle
    motor_protection_step(&drv.protection, to_ms_since_boot(start));
    
    // Command the motor directly (inversion already applied)
    drv8833_cmd(&drv, u);
    
    // Emergency brake detection - force IDLE state if emergency brake is active
    static bool was_emergency_brake = false;
    bool is_emergency_brake = drv8833_is_emergency_brake_active(&drv);
    if (is_emergency_brake && !was_emergency_brake) {
        S.state = ST_IDLE;  // Force to idle state
        if (debug_is_output_enabled()) {
            printf("EMERGENCY BRAKE ACTIVATED - MOTOR PROTECTION ENGAGED!\n");
        }
    }
    was_emergency_brake = is_emergency_brake;
    
    // Debug: Print control command occasionally with detailed energy info
    static int debug_cmd_div = 0;
    if ((debug_cmd_div++ % 100) == 0 && debug_is_output_enabled()) {  // More frequent - every 100ms
        // Calculate energy for debugging
        float J = CALCULATE_MOMENT_OF_INERTIA(P.m, P.L, P.Jm); // From config.h
        float E = 0.5f * J * (S.omega * S.omega) + P.m * GRAVITY_ACCEL * (P.L * 0.5f) * (1.0f + cosf(S.theta_b));
        float Edes = CALCULATE_ENERGY_TARGET(P.m, P.L); // From config.h
        float e_diff = E - Edes;
        float control_sign = (S.omega * cosf(S.theta_b) >= 0) ? 1.0f : -1.0f;
        
        printf("CTRL: st=%d u=%.3f th_b=%.2f w=%.2f E=%.4f Ed=%.4f de=%.4f sign=%.1f prot=%d emrg=%d nrg=%.3f\n", 
               S.state, u, S.theta_b, S.omega, E, Edes, e_diff, control_sign, 
               drv8833_is_protection_active(&drv) ? 1 : 0, 
               drv8833_is_emergency_brake_active(&drv) ? 1 : 0, 
               drv8833_get_protection_energy_used(&drv));
               
        // Additional calibration verification - show raw angles vs processed
        if (debug_cmd_div % 500 == 0) {  // Every 5 seconds
            printf("CALIB CHECK: raw=%.3f, offset=%.3f, theta_b=%.3f (hanging should be ~0, upright should be ~π=%.3f)\n",
                   theta_raw, enc.angle_offset_rad, S.theta_b, (float)M_PI);
        }
    }

    // 4) Blink and log
    static int led_div = 0;
    if ((led_div++ % 100) == 0) {
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
    }

    // Logging at much lower rate - ~1 Hz instead of 100 Hz, only if debug enabled
    static int log_div = 0;
    if ((log_div++ % 1000) == 0) {  // Changed from % 10 to % 1000
        // Only log if debug output is enabled
        if (debug_is_output_enabled()) {
            uint8_t agc=0; uint16_t mag=0;
            as5600_read_agc_mag(&enc, &agc, &mag);
            
            // Calculate and display energy for debugging
            float J = (P.m * P.L * P.L) / 3.0f + P.Jm;
            float E = 0.5f * J * (S.omega * S.omega) + P.m * GRAVITY_ACCEL * (P.L * 0.5f) * (1.0f + cosf(S.theta_b));
            float Edes = CALCULATE_ENERGY_TARGET(P.m, P.L); // From config.h
            float e_diff = E - Edes;
            
            printf("%.3f,%d,%.6f,%.6f,%.6f,%.4f,%.6f,%u,%u,%u,E=%.6f,Ed=%.6f,e=%.6f\r\n",
                tsec,
                S.state,
                S.theta_b,
                S.theta_u,
                S.omega,
                S.u,
                theta_raw,
                (unsigned)agc,
                (unsigned)mag,
                encoder_ok ? 1u : 0u,
                E, Edes, e_diff
            );
        }
    }

    tsec += dt;
    in_cb = false;
    return true; // repeat
}

int main() {

    // Initialize stdio for USB
    stdio_init_all();

    // Initialize GPIO for LED first
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);  // LED on
    
    // Wait a bit for USB
    sleep_ms(1000);
    
    // Initialize debug system first so we can check if output is enabled
    debug_init();
    
    // Only print startup message if debug is enabled
    if (debug_is_output_enabled()) {
        printf("\n\n=== Pico Pendulum Starting ===\n");
        printf("Debug system active - ready for commands\n");
    }
    
    // Set USB ready flag since we can clearly print
    debug_set_state(DBG_FLAG_USB_READY);
    
    // Blink LED 3 times slowly
    for (int i = 0; i < 3; i++) {
        gpio_put(LED_PIN, 0);
        sleep_ms(200);
        gpio_put(LED_PIN, 1);
        sleep_ms(200);
    }
    
    // Initialize watchdog to catch crashes (longer timeout during init)
    watchdog_enable(15000, 1);  // 15 second timeout during initialization
    
    if (debug_is_output_enabled()) {
        printf("Debug system initialized\n");
    }
    
    // Initialize I2C with comprehensive error checking
    if (debug_is_output_enabled()) {
        printf("Initializing I2C1 on GPIO2/3...\n");
    }
    
    // Initialize I2C at a conservative speed for reliability
    uint32_t actual_baud = i2c_init(i2c1, 100000);  // Use i2c1 instead of i2c0
    if (actual_baud == 0) {
        if (debug_is_output_enabled()) {
            printf("WARNING: I2C1 initialization returned 0 baud - trying to continue anyway\n");
        }
        // Try a different baud rate
        actual_baud = i2c_init(i2c1, 100000);  // Try slower speed
        if (actual_baud == 0) {
            if (debug_is_output_enabled()) {
                printf("WARNING: I2C1 completely failed to initialize - continuing without I2C\n");
            }
            // Don't hang - continue without I2C
        }
    }
    if (debug_is_output_enabled()) {
        printf("I2C1 initialized at %lu Hz (requested 400000 Hz)\n", actual_baud);
    }
    
    // Set GPIO functions for I2C (no strict verification to avoid hangs)
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    
    // Check GPIO functions but don't hang if verification fails
    if (gpio_get_function(I2C_SDA) != GPIO_FUNC_I2C || gpio_get_function(I2C_SCL) != GPIO_FUNC_I2C) {
        if (debug_is_output_enabled()) {
            printf("WARNING: GPIO functions may not be set correctly for I2C1\n");
            printf("SDA function: %d (expected %d), SCL function: %d (expected %d)\n", 
                   gpio_get_function(I2C_SDA), GPIO_FUNC_I2C, 
                   gpio_get_function(I2C_SCL), GPIO_FUNC_I2C);
        }
        // Continue anyway - don't hang the system
    }
    
    // Enable internal pull-ups (essential for I2C)
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Set drive strength to help with signal integrity
    gpio_set_drive_strength(I2C_SDA, GPIO_DRIVE_STRENGTH_12MA);
    gpio_set_drive_strength(I2C_SCL, GPIO_DRIVE_STRENGTH_12MA);
    
    if (debug_is_output_enabled()) {
        printf("I2C1 GPIOs configured: SDA=GPIO%d, SCL=GPIO%d with 12mA drive strength\n", I2C_SDA, I2C_SCL);
    }
    
    // Extended delay for I2C bus to stabilize (with debug command processing)
    if (debug_is_output_enabled()) {
        printf("Waiting for I2C1 bus stabilization...\n");
    }
    
    // SAFETY: Process debug commands during I2C stabilization delay
    for (int i = 0; i < 100; i++) {  // 100 x 10ms = 1000ms total
        sleep_ms(10);
        debug_process();  // Always process debug commands
        watchdog_update();  // Update watchdog
    }
    
    // Test I2C bus integrity with detailed diagnostics (with timeout protection)
    if (debug_is_output_enabled()) {
        printf("Testing I2C bus integrity...\n");
    }
    
    // Try a simple bus scan to see if we can even communicate (limited scan to prevent hanging)
    bool i2c_working = false;
    int devices_found = 0;
    
    // SAFETY: Only scan a few key addresses to prevent long hangs
    uint8_t test_addresses[] = {0x36, 0x48, 0x68, 0x77}; // AS5600, ADS, RTC, BME280
    for (int addr_idx = 0; addr_idx < 4; addr_idx++) {
        uint8_t test_addr = test_addresses[addr_idx];
        uint8_t test_data;
        // Use very short timeout to prevent hanging
        int result = i2c_read_timeout_us(i2c1, test_addr, &test_data, 1, false, 1000);  // 1ms timeout only
        if (result >= 0) {
            if (debug_is_output_enabled()) {
                printf("I2C1: Device found at address 0x%02X\n", test_addr);
            }
            devices_found++;
            i2c_working = true;
            
            // Special handling for AS5600
            if (test_addr == 0x36) {
                if (debug_is_output_enabled()) {
                    printf("I2C1: AS5600 detected during bus scan!\n");
                }
            }
        }
        // Safety: update watchdog even during I2C scan
        watchdog_update();
    }
    
    if (!i2c_working) {
        if (debug_is_output_enabled()) {
            printf("WARNING: I2C1 bus completely unresponsive!\n");
            printf("Hardware checklist:\n");
            printf("  1. Wiring: SDA=GPIO2, SCL=GPIO3 (changed from GPIO4/5)\n");
            printf("  2. External pull-up resistors (1.8k-4.7k ohm to 3.3V)\n");
            printf("  3. AS5600 power supply (3.3V or 5V)\n");
            printf("  4. Ground connection between Pico and AS5600\n");
            printf("  5. Check for loose connections\n");
            printf("  6. Verify AS5600 is not damaged\n");
        }
    } else {
        if (debug_is_output_enabled()) {
            printf("I2C1: Bus responding - found %d device(s)\n", devices_found);
        }
    }
    
    // Comprehensive I2C device scan (only if debug enabled to prevent hangs)
    if (debug_is_output_enabled()) {
        printf("Performing comprehensive I2C1 device scan...\n");
        debug_scan_i2c(i2c1);
    }
    
    // Try to initialize AS5600 sensor (with timeout protection)
    if (debug_is_output_enabled()) {
        printf("Initializing AS5600 sensor on I2C1...\n");
    }
    enc_found = false;
    
    // SAFETY: Limit retries and add timeouts to prevent hanging
    for (int retry = 0; retry < 2; retry++) {  // Reduced from 3 to 2 retries
        if (debug_is_output_enabled()) {
            printf("AS5600 initialization attempt %d/2...\n", retry + 1);
        }
        
        // Safety: update watchdog before each attempt
        watchdog_update();
        
        // Pass baud rate but as5600_init won't use it (already configured)
        if (as5600_init(&enc, i2c1, I2C_SDA, I2C_SCL, 100000)) {
            enc_found = true;
            if (debug_is_output_enabled()) {
                printf("AS5600 initialized successfully on I2C1!\n");
            }
            debug_set_state(DBG_FLAG_I2C_OK);
            break;
        } else {
            if (debug_is_output_enabled()) {
                printf("AS5600 initialization failed on attempt %d\n", retry + 1);
            }
            if (retry < 1) {  // Only sleep once between retries
                if (debug_is_output_enabled()) {
                    printf("Retrying in 100ms...\n");
                }
                sleep_ms(100);  // Reduced from 200ms to 100ms
                watchdog_update();  // Safety: update watchdog during sleep
            }
        }
    }
    
    if (!enc_found) {
        if (debug_is_output_enabled()) {
            printf("WARNING: AS5600 not found - continuing without encoder\n");
        }
    }
    
    // Initialize motor driver with built-in protection - OPTIMIZED FOR HIGH-FRICTION MOTORS
    if (debug_is_output_enabled()) {
        printf("Initializing motor driver...\n");
    }
    // Use HIGH PWM frequency for smooth motor operation - optimal for brushed motors
    drv8833_init(&drv, PWM_IN1, PWM_IN2, 1000);  // 1 kHz PWM - above audible range, good control
    debug_set_state(DBG_FLAG_MOTOR_OK);  // Set motor OK flag

    
    // Initialize control parameters with safe defaults
    if (debug_is_output_enabled()) {
        printf("Initializing control parameters...\n");
    }
    ctrl_init(&P, &S);
    
    // Control params (tune to your build) - CONSERVATIVE VALUES for safety
    P.m = 0.2f;      // 200g rod mass (was 20g - likely too light)
    P.L = 0.3048f;   // 12 inches = 30.48cm length
    P.Jm = 1e-5f;    // motor+hub inertia (rough)
    P.b_vis = 0.0002f; // small viscous friction
    P.tau_ff = 0.001f; // Coulomb friction bias (N*m) - MATCH PC VERSION
    P.tau_max = 0.05f; // available peak torque at driver limit (N*m) - MATCH PC VERSION  
    P.u_to_tau = 0.05f; // N*m at u=1.0 (tune) - MATCH PC VERSION

    P.dt = 1.0f / CONTROL_HZ;

    // Swing-up - Allow full power for effective swing-up
    P.k_energy = 1.0f; // Energy pump gain - MATCH PC VERSION
    P.swing_sat = SWING_SATURATION; // Max motor power for swing-up from config.h

    // Balance (PD+I) - REDUCED gains for stability
    P.Kp = 5.0f;     // REDUCED from 18.0 to 5.0
    P.Kd = 1.0f;     // REDUCED from 2.5 to 1.0
    P.Ki = 0.1f;     // REDUCED from 0.35 to 0.1
    P.ui_max = 0.2f; // REDUCED integral limit
    P.balance_sat = BALANCE_SATURATION; // Max power from config.h

    // Handover thresholds - MATCH PC VERSION EXACTLY
    P.theta_catch = 30.0f * 3.14159f / 180.0f; // 30 degrees from upright (PC version)
    // Tighten the speed threshold for transition to balance to match the PC simulator.
    // A 3 rad/s catch zone prevents declaring balance too early when the pendulum is still moving quickly.
    P.omega_catch = 0.5f; // 0.5 rad/s threshold (PC version well_settled threshold)
    
    // Braking parameters from config.h
    P.brake_light = BRAKE_LIGHT;         // Light braking for swing-up transitions
    P.brake_moderate = BRAKE_MODERATE;   // Moderate braking for high speeds
    P.brake_strong = BRAKE_STRONG;       // Strong braking for very high speeds
    P.brake_maximum = BRAKE_MAXIMUM;     // Maximum braking for extreme speeds
    
    // Set desired energy using EXACT PC implementation formula
    // PC: state.Edes = params.m * 9.81f * (params.L * 0.5f) * 2.2f;
    S.Edes = P.m * 9.81f * (P.L * 0.5f) * 2.2f; // EXACT MATCH to working PC version
    
    // CONDITIONAL filtering - lighter for swing-up, heavier for balance
    // Swing-up needs responsive signals, balance needs smooth signals
    lpf1_init(&lp_angle, 0.92f, 0.0f);  // Moderate filtering (compromise between PC and ultra-heavy)
    diff_lpf_init(&dtheta, 0.75f); // Moderate filtering on derivative
    
    // CRITICAL FIX: Initialize the Kalman filter here, ONCE.
    float J = (P.m * P.L * P.L) / 3.0f + P.Jm;
    // CORRECTED: For a rod pivoted at one end with center of mass at L/2,
    // linearized around upright (unstable): theta_u'' = -a*theta_u + b*u
    // where a = (m*g*L/2)/J for the gravity term (NEGATIVE for unstable equilibrium)
    float a = -(P.m * 9.81f * (P.L / 2.0f)) / J;  // FIXED: Negative sign for inverted pendulum
    float b = P.u_to_tau / J;  // FIXED: Incorporate MOTOR_INVERT into u_to_tau instead
    
    // If MOTOR_INVERT is true, adjust the sign of b so Kalman sees the correct torque
    if (MOTOR_INVERT) {
        b = -b;
    }
    
    // ULTRA-AGGRESSIVE Kalman filtering for high-precision balance control on embedded hardware
    // Microcontroller environment requires much heavier filtering due to sensor noise and discrete effects
    kalman_init(&KF, P.dt, a, b, 1e-5f, 5e-4f, 2e-4f);  // Industry-grade servo filtering for embedded systems
    
    // Skip calibration if no encoder, but still set up control
    if (enc_found) {
        if (debug_is_output_enabled()) {
            printf("Running encoder calibration...\n");
            printf("Please position pendulum at bottom (hanging down) and hold steady...\n");
        }
        
        // ROBUST CALIBRATION: Use sine/cosine averaging to handle wrap-around angles
        float sum_sin = 0.0f;
        float sum_cos = 0.0f;
        int cal_count = 0;
        int max_cal_attempts = 30;  // Reduced from 50 to 30 for faster boot
        
        for (int i = 0; i < max_cal_attempts; i++) {
            gpio_put(LED_PIN, i & 1);
            sleep_ms(100);
            watchdog_update();  // SAFETY: Always update watchdog during calibration
            
            float angle_raw;
            // SAFETY: Don't hang if encoder fails - skip this reading
            if (as5600_read_angle_rad(&enc, &angle_raw)) {
                // Apply sensor inversion if configured (FIXED: honor SENSOR_INVERT)
                if (SENSOR_INVERT) {
                    angle_raw = -angle_raw;
                }
                
                // Accumulate sine and cosine for robust circular mean
                sum_sin += sinf(angle_raw);
                sum_cos += cosf(angle_raw);
                cal_count++;
                
                // Print progress less frequently to avoid USB issues
                if ((i % 10 == 0) && debug_is_output_enabled()) {  // Every 1 second instead of every 2
                    printf("Calibration: %d/%d (got %d readings)\n", i, max_cal_attempts, cal_count);
                }
                
                // EARLY EXIT: If we have enough good readings, don't wait for full loop
                if (cal_count >= 15) {  // Exit early if we have 15 good readings
                    if (debug_is_output_enabled()) {
                        printf("Calibration: Early completion with %d readings\n", cal_count);
                    }
                    break;
                }
            } else {
                // If encoder read fails, don't count it but continue trying
                if (debug_is_output_enabled() && (i % 10 == 0)) {
                    printf("Calibration: Encoder read failed at step %d\n", i);
                }
            }
        }
        
        if (cal_count > 3) {  // Reduced threshold from 5 to 3 for more lenient success
            // Calculate circular mean using atan2 - handles wrap-around correctly
            enc.angle_offset_rad = atan2f(sum_sin, sum_cos);
            
            // Test the calibration by reading current angle
            float test_angle_raw;
            if (as5600_read_angle_rad(&enc, &test_angle_raw)) {
                // Apply sensor inversion to test reading too
                if (SENSOR_INVERT) {
                    test_angle_raw = -test_angle_raw;
                }
                
                float theta_b_test = wrap_pi(test_angle_raw - enc.angle_offset_rad);
                
                if (debug_is_output_enabled()) {
                    printf("Calibration test: raw=%.3f, offset=%.3f, theta_b=%.3f\n", 
                           test_angle_raw, enc.angle_offset_rad, theta_b_test);
                    printf("ROBUST CALIBRATION: No π correction needed - offset points to hanging position\n");
                }
            }
            
            if (debug_is_output_enabled()) {
                printf("Calibration OK: final offset = %.3f rad (from %d readings, circular mean)\n", 
                       enc.angle_offset_rad, cal_count);
                printf("Sensor inversion: %s\n", SENSOR_INVERT ? "ENABLED" : "disabled");
            }
            debug_set_state(DBG_FLAG_CALIB_DONE);
            
            // AUTO-START: Start in SWING-UP mode automatically if encoder is working
            S.state = ST_SWINGUP;
            if (debug_is_output_enabled()) {
                printf("Control state set to SWING-UP (AUTO-START MODE)\n");
            }
            
            // Visual feedback: Flash LED rapidly to indicate auto-start mode
            for (int i = 0; i < 6; i++) {
                gpio_put(LED_PIN, 1);
                sleep_ms(100);
                gpio_put(LED_PIN, 0); 
                sleep_ms(100);
            }
        } else {
            if (debug_is_output_enabled()) {
                printf("Calibration failed - continuing with swing-up anyway\n");
                printf("WARNING: No calibration offset - using zero offset\n");
            }
            enc.angle_offset_rad = 0.0f;
            // MODIFIED: Still try swing-up even without calibration
            S.state = ST_SWINGUP;
            if (debug_is_output_enabled()) {
                printf("Control state set to SWING-UP (UNCALIBRATED MODE)\n");
            }
        }
    } else {
        if (debug_is_output_enabled()) {
            printf("No encoder - but still attempting swing-up with open-loop control\n");
        }
        // MODIFIED: Try swing-up even without encoder
        S.state = ST_SWINGUP;
        if (debug_is_output_enabled()) {
            printf("Control state set to SWING-UP (OPEN-LOOP MODE)\n");
        }
    }
    
    // Main control loop
    if (debug_is_output_enabled()) {
        printf("Starting main loop...\n");
    }
    
    // Create timer for control loop
    struct repeating_timer timer;
    bool timer_running = false;
    
    // Start with control disabled for safety
    control_enabled = false;
    
    // Add timer with error checking
    timer_running = add_repeating_timer_ms(-1, repeating_timer_cb, NULL, &timer);
    if (!timer_running) {
        if (debug_is_output_enabled()) {
            printf("ERROR: Failed to create timer!\n");
        }
    }
    
    // Enable control after a delay
    sleep_ms(500);
    control_enabled = true;
    debug_set_state(DBG_FLAG_RUNNING);  // Set running flag
    
    // AUTO-START: Give initial perturbation if we're in swing-up mode and have encoder
    if (enc_found && S.state == ST_SWINGUP) {
        if (debug_is_output_enabled()) {
            printf("AUTO-START: Applying initial perturbation to start motion...\n");
        }
        
        // Stronger perturbation to get motion started
        sleep_ms(1000);  // Wait a moment for system to stabilize
        drv8833_cmd(&drv, 0.3f);  // STRONGER forward pulse
        sleep_ms(250);  // LONGER pulse duration
        watchdog_update();
        drv8833_cmd(&drv, -0.3f); // STRONGER reverse pulse  
        sleep_ms(250);  // LONGER pulse duration
        watchdog_update();
        drv8833_cmd(&drv, 0.0f);  // Let control system take over
        
        if (debug_is_output_enabled()) {
            printf("AUTO-START: Initial perturbation complete - control system active\n");
        }
    }
    
    // Reduce watchdog timeout now that initialization is complete
    watchdog_enable(8000, 1);  // 8 second timeout during normal operation
    
    if (debug_is_output_enabled()) {
        printf("Control enabled\n");
    }
    
    // Main loop with watchdog
    uint32_t loop_count = 0;
    uint32_t last_heartbeat = 0;
    
    if (debug_is_output_enabled()) {
        printf("Main loop started. Press 'h' for help.\n");
    }
    
    while (1) {
        // Update watchdog
        watchdog_update();
        
        // Process debug commands more frequently
        debug_process();
        
        // Heartbeat much less frequently - every 60 seconds, only if debug enabled
        if (loop_count % 6000 == 0 && loop_count != last_heartbeat && debug_is_output_enabled()) {
            printf("Loop %lu, enc=%d, ctrl=%d\n", 
                   loop_count, enc_found, control_enabled);
            last_heartbeat = loop_count;
        }
        
        loop_count++;
        
        // Longer sleep to give more time for USB processing
        sleep_ms(10);  // 10ms instead of 1ms
    }
    
    return 0;
}
