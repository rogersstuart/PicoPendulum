#ifndef DEBUG_H
#define DEBUG_H

#include <stdbool.h>

#ifdef PICO_BUILD
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#else
// PC/Qt build - provide minimal definitions
#include <stdio.h>
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

// Debug state flags
#define DBG_FLAG_USB_READY    (1 << 0)
#define DBG_FLAG_I2C_OK       (1 << 1)
#define DBG_FLAG_MOTOR_OK     (1 << 2)
#define DBG_FLAG_CALIB_DONE   (1 << 3)
#define DBG_FLAG_RUNNING      (1 << 4)
#define DBG_FLAG_ERROR        (1 << 5)

// Interrupt-driven debug system
#define DEBUG_CMD_QUEUE_SIZE 8
#define DEBUG_CMD_BUFFER_SIZE 64

// Command queue for interrupt-driven processing
typedef struct {
    char cmd[DEBUG_CMD_BUFFER_SIZE];
    bool ready;
    uint32_t timestamp_ms;
} debug_cmd_entry_t;

// Motor control override flags (atomic)
typedef struct {
    volatile bool motor_override_active;    // Debug has motor control
    volatile float motor_override_value;    // Motor command value (-1.0 to 1.0)
    volatile uint32_t motor_override_timeout_ms; // Timeout for safety
    volatile bool control_loop_enabled;     // Control loop state
    volatile bool emergency_stop;           // Emergency stop flag
} debug_motor_state_t;

// Command codes
typedef enum {
    CMD_NONE = 0,
    CMD_STATUS,      // Print all status info
    CMD_START,       // Start control loop
    CMD_STOP,        // Stop control loop
    CMD_PARAMS,      // Print/set parameters
    CMD_CALIBRATE,   // Run calibration
    CMD_DEBUG_ON,    // Enable debug output
    CMD_DEBUG_OFF,   // Disable debug output
    CMD_HELP,        // Print command help
} debug_cmd_t;

// LED patterns
typedef enum {
    LED_OFF,
    LED_ON,
    LED_SLOW_BLINK,    // 1 Hz - Idle/Ready
    LED_FAST_BLINK,    // 5 Hz - Error
    LED_PULSE,         // Brief flash - Running
    LED_DOUBLE_BLINK,  // Calibrating
    LED_TRIPLE_BLINK,  // I2C Error (3 blinks then pause)
} led_pattern_t;

#ifdef PICO_BUILD
// Initialize debug interface with interrupt support
void debug_init(void);

// Initialize UART interrupt system
void debug_init_uart_interrupt(void);

// Process any pending commands (deferred execution)
void debug_process(void);

// Get motor override state (for control loop coordination)
debug_motor_state_t* debug_get_motor_state(void);

// Check if debug has motor control (called by control loop)
bool debug_has_motor_control(float* motor_value);

// Emergency stop from any context
void debug_emergency_stop(void);

// Set LED pattern
void debug_set_led(led_pattern_t pattern);

// Update system state flags
void debug_set_state(uint32_t flags);
void debug_clear_state(uint32_t flags);

// Print status message with timestamp
void debug_print(const char* fmt, ...);

// Check if debug output is enabled
bool debug_is_output_enabled(void);

// Check if quiet mode is enabled
bool debug_is_quiet_mode(void);

// Global variables that debug module needs access to
extern volatile bool enc_found;
extern bool thermal_protection_active;

// Motor control commands (deferred execution)
void debug_toggle_motor(void);
void debug_test_motor(void);
void debug_test_motor_custom(float power_percent, float duration_sec);
void debug_toggle_pwm_mode(void);
void debug_pwm_test(void);
void debug_pwm_ramp(void);
void debug_wake_pendulum(void);
void debug_reset_thermal(void);

// Encoder monitoring
void debug_show_encoder_realtime(void);
void debug_test_direction(void);
void debug_test_encoder_direction(void);
void debug_test_motor_encoder_sign_match(void);
void debug_recalibrate_down(void);

// Scan I2C bus for devices, particularly looking for AS5600 at 0x36
void debug_scan_i2c(i2c_inst_t *i2c);
#else
// PC/Qt build - provide stub functions
static inline void debug_init(void) {}
static inline void debug_process(void) {}
static inline bool debug_has_motor_control(float* motor_value) { (void)motor_value; return false; }
static inline bool debug_is_output_enabled(void) { return false; }
static inline bool debug_is_quiet_mode(void) { return true; }
static inline void debug_print(const char* fmt, ...) { (void)fmt; }
#endif

#ifdef __cplusplus
}
#endif

#endif
