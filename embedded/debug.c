#include "debug.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#include "as5600.h"
#include "drv8833.h"
#include "control.h"
#include "filters.h"

// Ensure M_PI is defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper function for angle wrapping
static inline float wrap_pi(float x) {
    while (x > M_PI)  x -= 2*M_PI;
    while (x <= -M_PI) x += 2*M_PI;
    return x;
}

// External variables from main.c
extern as5600_t enc;
extern drv8833_t drv;
// enc_found is now declared in debug.h
extern volatile bool control_enabled;
// thermal_protection_active is now declared in debug.h
extern bool MOTOR_INVERT;
extern bool SENSOR_INVERT;
extern ctrl_state_t S;
extern ctrl_params_t P;
extern lpf1_t lp_angle;
extern diff_lpf_t dtheta;

// Interrupt-driven debug system state
static debug_cmd_entry_t cmd_queue[DEBUG_CMD_QUEUE_SIZE];
static volatile uint32_t cmd_queue_head = 0;
static volatile uint32_t cmd_queue_tail = 0;
static volatile uint32_t cmd_queue_count = 0;

// Motor control coordination (atomic access)
static debug_motor_state_t motor_state = {
    .motor_override_active = false,
    .motor_override_value = 0.0f,
    .motor_override_timeout_ms = 0,
    .control_loop_enabled = true,
    .emergency_stop = false
};

// Command input buffer for interrupt handler
static char uart_input_buffer[DEBUG_CMD_BUFFER_SIZE];
static volatile uint32_t uart_buffer_index = 0;

static uint32_t system_state = 0;
static led_pattern_t current_led = LED_OFF;
static bool debug_output_enabled = false;  // Disable debug output by default (periodic/error messages)
static bool quiet_mode = false;            // Quiet mode OFF by default (command responses enabled)
static absolute_time_t last_led_toggle;
static bool led_state = false;
static uint32_t led_counter = 0;

// LED timing constants
#define LED_SLOW_MS 500
#define LED_FAST_MS 100
#define LED_PULSE_MS 50
#define LED_DOUBLE_BLINK_MS 200

// UART interrupt handler - processes characters as they arrive
static void uart_irq_handler(void) {
    // Check if RX FIFO has data
    while (uart_is_readable(uart0)) {
        uint8_t ch = uart_getc(uart0);
        
        // Handle special characters immediately for responsiveness
        if (ch == 0x03) {  // Ctrl+C - Emergency stop
            motor_state.emergency_stop = true;
            motor_state.motor_override_active = false;
            motor_state.motor_override_value = 0.0f;
            // Send immediate response without printf (avoid recursion)
            uart_putc_raw(uart0, '\n');
            uart_puts(uart0, "EMERGENCY STOP!\n");
            return;
        }
        
        // Handle backspace
        if (ch == 0x08 || ch == 0x7F) {
            if (uart_buffer_index > 0) {
                uart_buffer_index--;
                uart_putc_raw(uart0, 0x08);  // Backspace
                uart_putc_raw(uart0, ' ');   // Space
                uart_putc_raw(uart0, 0x08);  // Backspace again
            }
            continue;
        }
        
        // Echo character for user feedback
        if (ch >= ' ' && ch <= '~') {  // Printable characters
            uart_putc_raw(uart0, ch);
        }
        
        // Handle line endings
        if (ch == '\n' || ch == '\r') {
            uart_putc_raw(uart0, '\n');  // Always send newline for consistency
            
            // Null-terminate the command
            uart_input_buffer[uart_buffer_index] = '\0';
            
            // Only queue non-empty commands
            if (uart_buffer_index > 0) {
                // Check if queue has space
                if (cmd_queue_count < DEBUG_CMD_QUEUE_SIZE) {
                    // Add command to queue
                    uint32_t next_head = (cmd_queue_head + 1) % DEBUG_CMD_QUEUE_SIZE;
                    strncpy(cmd_queue[cmd_queue_head].cmd, uart_input_buffer, DEBUG_CMD_BUFFER_SIZE - 1);
                    cmd_queue[cmd_queue_head].cmd[DEBUG_CMD_BUFFER_SIZE - 1] = '\0';
                    cmd_queue[cmd_queue_head].ready = true;
                    cmd_queue[cmd_queue_head].timestamp_ms = to_ms_since_boot(get_absolute_time());
                    
                    cmd_queue_head = next_head;
                    cmd_queue_count++;
                } else {
                    // Queue full - send error message
                    uart_puts(uart0, "CMD_QUEUE_FULL\n");
                }
            }
            
            // Reset buffer for next command
            uart_buffer_index = 0;
            continue;
        }
        
        // Add character to buffer if there's space
        if (uart_buffer_index < DEBUG_CMD_BUFFER_SIZE - 1) {
            uart_input_buffer[uart_buffer_index++] = ch;
        } else {
            // Buffer full - send error and reset
            uart_puts(uart0, "\nCMD_TOO_LONG\n");
            uart_buffer_index = 0;
        }
    }
}

// Initialize UART interrupt system
void debug_init_uart_interrupt(void) {
    // Initialize command queue
    for (int i = 0; i < DEBUG_CMD_QUEUE_SIZE; i++) {
        cmd_queue[i].ready = false;
        cmd_queue[i].cmd[0] = '\0';
    }
    
    // Reset motor control state
    motor_state.motor_override_active = false;
    motor_state.motor_override_value = 0.0f;
    motor_state.motor_override_timeout_ms = 0;
    motor_state.control_loop_enabled = true;
    motor_state.emergency_stop = false;
    
    // Set up UART0 interrupt
    irq_set_exclusive_handler(UART0_IRQ, uart_irq_handler);
    irq_set_enabled(UART0_IRQ, true);
    
    // Enable UART RX interrupt (but not TX to avoid conflicts with printf)
    uart_set_irq_enables(uart0, true, false);
}

// Motor control coordination functions
debug_motor_state_t* debug_get_motor_state(void) {
    return &motor_state;
}

bool debug_has_motor_control(float* motor_value) {
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    
    // Check for emergency stop
    if (motor_state.emergency_stop) {
        *motor_value = 0.0f;
        return true;  // Debug has control to enforce stop
    }
    
    // Check if motor override is active and not timed out
    if (motor_state.motor_override_active) {
        if (motor_state.motor_override_timeout_ms > 0 && 
            current_time > motor_state.motor_override_timeout_ms) {
            // Timeout - release control
            motor_state.motor_override_active = false;
            motor_state.motor_override_value = 0.0f;
            motor_state.motor_override_timeout_ms = 0;
            return false;
        }
        
        *motor_value = motor_state.motor_override_value;
        return true;
    }
    
    return false;
}

void debug_emergency_stop(void) {
    motor_state.emergency_stop = true;
    motor_state.motor_override_active = false;
    motor_state.motor_override_value = 0.0f;
    motor_state.motor_override_timeout_ms = 0;
}

// Set motor override with timeout for safety
static void debug_set_motor_override(float value, uint32_t timeout_ms) {
    motor_state.motor_override_value = value;
    motor_state.motor_override_timeout_ms = to_ms_since_boot(get_absolute_time()) + timeout_ms;
    motor_state.motor_override_active = true;
    motor_state.emergency_stop = false;  // Clear emergency stop if setting new command
}

// Clear motor override
static void debug_clear_motor_override(void) {
    motor_state.motor_override_active = false;
    motor_state.motor_override_value = 0.0f;
    motor_state.motor_override_timeout_ms = 0;
}

// Forward declaration for command processing
static void process_debug_command(const char* cmd_buf);

void debug_init(void) {
    // Initialize LED GPIO (assuming LED_PIN is defined in main.c)
    gpio_init(25);  // Built-in LED
    gpio_set_dir(25, true);
    gpio_put(25, 0);
    
    last_led_toggle = get_absolute_time();
    debug_set_led(LED_SLOW_BLINK);  // Start with slow blink indicating ready state
    
    // Initialize interrupt-driven command processing
    debug_init_uart_interrupt();
    
    // Don't print anything unless debug output is explicitly enabled
}

void debug_process(void) {
    // Process LED patterns
    absolute_time_t now = get_absolute_time();
    uint32_t diff_ms = absolute_time_diff_us(last_led_toggle, now) / 1000;
    
    switch (current_led) {
        case LED_OFF:
            gpio_put(25, 0);
            break;
        case LED_ON:
            gpio_put(25, 1);
            break;
        case LED_SLOW_BLINK:
            if (diff_ms >= LED_SLOW_MS) {
                led_state = !led_state;
                gpio_put(25, led_state);
                last_led_toggle = now;
            }
            break;
        case LED_FAST_BLINK:
            if (diff_ms >= LED_FAST_MS) {
                led_state = !led_state;
                gpio_put(25, led_state);
                last_led_toggle = now;
            }
            break;
        case LED_PULSE:
            if (diff_ms >= LED_PULSE_MS) {
                gpio_put(25, 0);  // Always return to off
                current_led = LED_OFF;
            }
            break;
        case LED_DOUBLE_BLINK:
            if (diff_ms >= LED_DOUBLE_BLINK_MS) {
                led_counter++;
                if (led_counter == 1 || led_counter == 3) {
                    gpio_put(25, 1);
                } else if (led_counter == 2 || led_counter == 4) {
                    gpio_put(25, 0);
                } else {
                    led_counter = 0;
                }
                last_led_toggle = now;
            }
            break;
    }
    
    // Process queued commands from interrupt handler (deferred execution)
    while (cmd_queue_count > 0) {
        // Get next command from queue
        debug_cmd_entry_t* cmd = &cmd_queue[cmd_queue_tail];
        if (!cmd->ready) {
            break;  // Command not ready yet
        }
        
        // Process the command
        process_debug_command(cmd->cmd);
        
        // Mark command as processed and advance queue
        cmd->ready = false;
        cmd_queue_tail = (cmd_queue_tail + 1) % DEBUG_CMD_QUEUE_SIZE;
        cmd_queue_count--;
    }
    
    // Handle motor override timeouts
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (motor_state.motor_override_active && 
        motor_state.motor_override_timeout_ms > 0 && 
        current_time > motor_state.motor_override_timeout_ms) {
        motor_state.motor_override_active = false;
        motor_state.motor_override_value = 0.0f;
        motor_state.motor_override_timeout_ms = 0;
        if (!quiet_mode) {
            printf("Motor override timeout - control returned to system\n");
        }
    }
}

// Process individual debug command (deferred execution from interrupt)
static void process_debug_command(const char* cmd_buf) {
    // Help menu
    if (strcmp(cmd_buf, "h") == 0 || strcmp(cmd_buf, "help") == 0) {
        if (!quiet_mode) {
            printf("\n=== Debug Commands ===\n");
            printf("h/help - Help (this list)\n");
            printf("s      - System status\n");
            printf("i      - Scan I2C1 bus\n");
            printf("e      - Show encoder position (realtime)\n");
            printf("dir    - Test encoder direction and motor polarity\n");
            printf("inv    - Toggle motor inversion\n");
            printf("sinv   - Toggle sensor inversion (encoder direction)\n");
            printf("encdir - Test encoder direction (move pendulum and observe)\n");
            printf("motorsign - Test motor sign match with encoder\n");
            printf("cal    - Recalibrate down position (hold pendulum at bottom)\n");
            printf("m      - Toggle motor enable/disable\n");
            printf("t      - Test motor (30%% power for 3 seconds each direction)\n");
            printf("test <power> <duration> - Test motor with custom power %% and duration in seconds\n");
            printf("mode   - Toggle PWM mode (sign-magnitude vs locked anti-phase)\n");
            printf("w      - Start swing-up mode (wake pendulum)\n");
            printf("r      - Reset thermal protection\n");
            printf("d      - Toggle debug output (periodic/error messages)\n");
            printf("q      - Toggle quiet mode (command responses)\n");
            printf("stop   - Emergency stop (same as Ctrl+C)\n");
            printf("set <param> <value> - Set control parameter (e.g. set Kp 20.0)\n");
            printf("params  - Show all control parameters\n");
            printf("restart - Restart control system (retain parameters)\n");
            printf("======================\n");
        }
    } else if (strcmp(cmd_buf, "stop") == 0) {
        debug_emergency_stop();
        if (!quiet_mode) {
            printf("EMERGENCY STOP ACTIVATED\n");
        }
    } else if (strcmp(cmd_buf, "s") == 0) {
        if (!quiet_mode) {
            printf("System state: 0x%08lx\n", system_state);
            printf("USB ready : %s\n", (system_state & DBG_FLAG_USB_READY) ? "YES" : "NO");
            printf("I2C OK    : %s\n", (system_state & DBG_FLAG_I2C_OK) ? "YES" : "NO");
            printf("Motor OK  : %s\n", (system_state & DBG_FLAG_MOTOR_OK) ? "YES" : "NO");
            printf("Calibrated: %s\n", (system_state & DBG_FLAG_CALIB_DONE) ? "YES" : "NO");
            printf("Running   : %s\n", (system_state & DBG_FLAG_RUNNING) ? "YES" : "NO");
            printf("Error     : %s\n", (system_state & DBG_FLAG_ERROR) ? "YES" : "NO");
            printf("Control   : %s\n", control_enabled ? "ENABLED" : "DISABLED");
            printf("Motor Inv : %s\n", MOTOR_INVERT ? "TRUE" : "FALSE");
            printf("Sensor Inv: %s\n", SENSOR_INVERT ? "TRUE" : "FALSE");
            printf("PWM Mode  : %s\n", drv.control_mode == DRV8833_SIGN_MAGNITUDE ? "SIGN-MAGNITUDE" : "LOCKED ANTI-PHASE");
            printf("Encoder   : %s\n", enc_found ? "FOUND" : "NOT FOUND");
            printf("Debug Override: %s", motor_state.motor_override_active ? "ACTIVE" : "INACTIVE");
            if (motor_state.motor_override_active) {
                printf(" (%.3f, timeout=%lu ms)", motor_state.motor_override_value, motor_state.motor_override_timeout_ms);
            }
            printf("\n");
            printf("Emergency Stop: %s\n", motor_state.emergency_stop ? "YES" : "NO");
            printf("Motor Protection: %s (Energy: %.2f/%.2f)\n", 
                   drv8833_is_protection_active(&drv) ? "ACTIVE" : "OK",
                   drv8833_get_protection_energy_used(&drv),
                   5.0f);  // MOTOR_MAX_ENERGY_BUDGET
            printf("Ctrl State: %d ", S.state);
            switch(S.state) {
                case ST_IDLE: printf("(IDLE)\n"); break;
                case ST_SWINGUP: printf("(SWING-UP)\n"); break;
                case ST_BALANCE: printf("(BALANCE)\n"); break;
                default: printf("(UNKNOWN)\n"); break;
            }
            if (enc_found) {
                printf("Angle: %.1f deg (%.3f rad)\n", S.theta_b * 57.3f, S.theta_b);
                printf("Velocity: %.3f rad/s\n", S.omega);
                printf("Energy: %.3f J (target: %.3f J)\n", S.E, S.Edes);
                printf("Motor cmd: %.3f\n", S.u);
            }
        }
    } else if (strcmp(cmd_buf, "i") == 0) {
        debug_scan_i2c(i2c1);
    } else if (strcmp(cmd_buf, "e") == 0) {
        debug_show_encoder_realtime();
    } else if (strcmp(cmd_buf, "dir") == 0) {
        debug_test_direction();
    } else if (strcmp(cmd_buf, "cal") == 0) {
        debug_recalibrate_down();
    } else if (strcmp(cmd_buf, "m") == 0) {
        debug_toggle_motor();
    } else if (strcmp(cmd_buf, "t") == 0) {
        debug_test_motor();
    } else if (strncmp(cmd_buf, "test ", 5) == 0) {
        // Parse custom motor test: "test 80 2" = 80% power for 2 seconds
        float power;
        float duration;
        if (sscanf(cmd_buf + 5, "%f %f", &power, &duration) == 2) {
            debug_test_motor_custom(power, duration);
        } else {
            if (!quiet_mode) {
                printf("Usage: test <power%%> <duration_seconds>\n");
                printf("Example: test 50 1.5  (50%% power for 1.5 seconds)\n");
            }
        }
    } else if (strcmp(cmd_buf, "inv") == 0) {
        MOTOR_INVERT = !MOTOR_INVERT;
        if (!quiet_mode) {
            printf("Motor invert set to %s\n", MOTOR_INVERT ? "TRUE" : "FALSE");
        }
    } else if (strcmp(cmd_buf, "sinv") == 0) {
        SENSOR_INVERT = !SENSOR_INVERT;
        if (!quiet_mode) {
            printf("Sensor invert set to %s\n", SENSOR_INVERT ? "TRUE" : "FALSE");
            printf("This affects encoder angle direction - test with 'encdir' command\n");
        }
    } else if (strcmp(cmd_buf, "encdir") == 0) {
        debug_test_encoder_direction();
    } else if (strcmp(cmd_buf, "motorsign") == 0) {
        debug_test_motor_encoder_sign_match();
    } else if (strcmp(cmd_buf, "mode") == 0) {
        debug_toggle_pwm_mode();
    } else if (strcmp(cmd_buf, "w") == 0) {
        debug_wake_pendulum();
    } else if (strcmp(cmd_buf, "r") == 0) {
        debug_reset_thermal();
    } else if (strcmp(cmd_buf, "d") == 0) {
        debug_output_enabled = !debug_output_enabled;
        if (!quiet_mode) {
            printf("Debug output %s\n", debug_output_enabled ? "ENABLED" : "DISABLED");
        }
    } else if (strcmp(cmd_buf, "q") == 0) {
        quiet_mode = !quiet_mode;
        printf("Quiet mode %s\n", quiet_mode ? "ENABLED" : "DISABLED");
    } else if (strncmp(cmd_buf, "set ", 4) == 0) {
        // Parse parameter setting: "set Kp 20.0"
        char param[32];
        float value;
        if (sscanf(cmd_buf + 4, "%s %f", param, &value) == 2) {
            bool found = false;
            if (strcmp(param, "Kp") == 0) { P.Kp = value; found = true; }
            else if (strcmp(param, "Kd") == 0) { P.Kd = value; found = true; }
            else if (strcmp(param, "Ki") == 0) { P.Ki = value; found = true; }
            else if (strcmp(param, "ui_max") == 0) { P.ui_max = value; found = true; }
            else if (strcmp(param, "swing_sat") == 0) { P.swing_sat = value; found = true; }
            else if (strcmp(param, "k_energy") == 0) { P.k_energy = value; found = true; }
            else if (strcmp(param, "m") == 0) { P.m = value; found = true; }
            else if (strcmp(param, "L") == 0) { P.L = value; found = true; }
            else if (strcmp(param, "Jm") == 0) { P.Jm = value; found = true; }
            
            if (found) {
                // Recalculate derived parameters
                S.Edes = P.m * 9.81f * P.L;  // Energy at the top
                if (!quiet_mode) {
                    printf("Set %s = %.3f\n", param, value);
                }
            } else {
                if (!quiet_mode) {
                    printf("Unknown parameter: %s\n", param);
                }
            }
        } else {
            if (!quiet_mode) {
                printf("Usage: set <param> <value>\n");
            }
        }
    } else if (strcmp(cmd_buf, "params") == 0) {
        if (!quiet_mode) {
            printf("Control Parameters:\n");
            printf("  Kp = %.3f (balance proportional gain)\n", P.Kp);
            printf("  Kd = %.3f (balance derivative gain)\n", P.Kd);
            printf("  Ki = %.3f (balance integral gain)\n", P.Ki);
            printf("  ui_max = %.3f (integral saturation)\n", P.ui_max);
            printf("  swing_sat = %.3f (swing-up saturation)\n", P.swing_sat);
            printf("  k_energy = %.3f (energy pump gain)\n", P.k_energy);
            printf("  m = %.3f kg (pendulum mass)\n", P.m);
            printf("  L = %.3f m (pendulum length)\n", P.L);
            printf("  Jm = %.6f kg⋅m² (motor inertia)\n", P.Jm);
            printf("  Edes = %.3f J (target energy)\n", S.Edes);
        }
    } else if (strcmp(cmd_buf, "restart") == 0) {
        // Reset control state but keep parameters
        motor_state.emergency_stop = false;
        motor_state.motor_override_active = false;
        motor_state.motor_override_value = 0.0f;
        motor_state.motor_override_timeout_ms = 0;
        
        control_enabled = false;
        S.state = ST_IDLE;
        S.theta_b = 0.f;
        S.theta_u = 0.f;
        S.omega = 0.f;
        S.u = 0.f;
        S.ui = 0.f;
        S.E = 0.f;
        drv8833_cmd(&drv, 0.0f);
        if (!quiet_mode) {
            printf("Control system restarted. State set to IDLE.\n");
        }
    } else if (strlen(cmd_buf) > 0) {
        if (!quiet_mode) {
            printf("Unknown command: %s\n", cmd_buf);
        }
    }
}

void debug_set_led(led_pattern_t pattern) {
    if (pattern != current_led) {
        current_led = pattern;
        led_counter = 0;
        last_led_toggle = get_absolute_time();
        
        // Initial state
        if (pattern == LED_PULSE) {
            gpio_put(25, 1);
        }
    }
}

void debug_set_state(uint32_t flags) {
    system_state |= flags;
}

void debug_clear_state(uint32_t flags) {
    system_state &= ~flags;
}

void debug_print(const char* fmt, ...) {
    if (!debug_output_enabled) return;
    
    va_list args;
    va_start(args, fmt);
    
    // Print timestamp
    uint32_t ms = to_ms_since_boot(get_absolute_time());
    printf("[%lu.%03lu] ", ms/1000, ms%1000);
    
    vprintf(fmt, args);
    va_end(args);
}

void debug_scan_i2c(i2c_inst_t *i2c) {
    printf("Scanning I2C1 bus (pins SDA=GPIO2, SCL=GPIO3)...\n");
    
    int found_count = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        int ret;
        uint8_t rxdata;
        
        // Use timeout to prevent hanging on bad devices
        ret = i2c_read_timeout_us(i2c, addr, &rxdata, 1, false, 5000);  // 5ms timeout
        
        if (ret >= 0) {
            printf("Found I2C device at address 0x%02X", addr);
            found_count++;
            
            if (addr == 0x36) {  // AS5600 address
                printf(" -> AS5600 magnetic sensor detected!");
                debug_set_state(DBG_FLAG_I2C_OK);
            }
            printf("\n");
        }
    }
    
    if (found_count == 0) {
        printf("No I2C devices found on I2C1. Hardware checklist:\n");
        printf("  - I2C1 wiring (SDA=GPIO2, SCL=GPIO3)\n");
        printf("  - Device power (3.3V or 5V for AS5600)\n");
        printf("  - Pull-up resistors (1.8k-4.7k ohm to 3.3V)\n");
        printf("  - Ground connections\n");
        printf("  - Verify AS5600 is functional\n");
    } else {
        printf("I2C1 scan complete - found %d device(s)\n", found_count);
    }
}

bool debug_is_output_enabled(void) {
    return debug_output_enabled;
}

bool debug_is_quiet_mode(void) {
    return quiet_mode;
}

void debug_toggle_motor(void) {
    control_enabled = !control_enabled;
    if (!quiet_mode) {
        printf("Motor control %s\n", control_enabled ? "ENABLED" : "DISABLED");
    }
    
    if (!control_enabled) {
        // Stop motor immediately when disabled using override
        debug_set_motor_override(0.0f, 1000);  // 1 second timeout
        S.state = ST_IDLE;
        if (!quiet_mode) {
            printf("Motor stopped, state set to IDLE\n");
        }
    } else {
        // Clear any debug override and enable control
        debug_clear_motor_override();
        motor_state.emergency_stop = false;  // Clear emergency stop when enabling
        
        if (enc_found) {
            S.state = ST_SWINGUP;
            if (!quiet_mode) {
                printf("Control enabled, state set to SWING-UP\n");
                printf("Pendulum will start moving when perturbed\n");
            }
        } else {
            S.state = ST_IDLE;
            if (!quiet_mode) {
                printf("No encoder found - staying in IDLE state\n");
            }
        }
    }
}

void debug_test_motor(void) {
    if (!quiet_mode) {
        printf("Testing motor: 30%% power for 3 seconds each direction...\n");
        printf("Motor inversion is now handled automatically in control system\n");
        printf("Forward (positive command)...\n");
    }
    
    // Use motor override for forward direction with 4 second timeout (safety)
    debug_set_motor_override(0.3f, 4000);
    
    // Sleep with watchdog feeding and progress indication - 3 seconds
    for (int i = 0; i < 30; i++) {
        if (!quiet_mode && i % 5 == 0) {
            printf(".");
            fflush(stdout);
        }
        sleep_ms(100);
        watchdog_update();
    }
    
    if (!quiet_mode) {
        printf("\nReverse (negative command)...\n");
    }
    
    // Use motor override for reverse direction with 4 second timeout (safety)
    debug_set_motor_override(-0.3f, 4000);
    
    // Sleep with watchdog feeding and progress indication - 3 seconds
    for (int i = 0; i < 30; i++) {
        if (!quiet_mode && i % 5 == 0) {
            printf(".");
            fflush(stdout);
        }
        sleep_ms(100);
        watchdog_update();
    }
    
    if (!quiet_mode) {
        printf("\nStop\n");
    }
    
    // Clear motor override to stop
    debug_clear_motor_override();
    
    if (!quiet_mode) {
        printf("Motor test complete\n");
    }
}

void debug_test_motor_custom(float power_percent, float duration_sec) {
    // Validate inputs
    if (power_percent < -100.0f || power_percent > 100.0f) {
        if (!quiet_mode) {
            printf("Error: Power must be between -100 and 100 percent\n");
        }
        return;
    }
    
    if (duration_sec <= 0.0f || duration_sec > 10.0f) {
        if (!quiet_mode) {
            printf("Error: Duration must be between 0.1 and 10 seconds\n");
        }
        return;
    }
    
    float power_value = power_percent / 100.0f;  // Convert to [-1, 1]
    uint32_t duration_ms = (uint32_t)(duration_sec * 1000.0f);
    
    if (!quiet_mode) {
        printf("Testing motor: %.1f%% power for %.1f seconds...\n", power_percent, duration_sec);
    }
    
    // Use motor override for safety
    debug_set_motor_override(power_value, duration_ms + 1000);  // +1s safety margin
    
    // Wait for specified duration with progress indication
    uint32_t steps = duration_ms / 100;  // 100ms per step
    for (uint32_t i = 0; i < steps; i++) {
        if (!quiet_mode && i % 5 == 0) {
            printf(".");
            fflush(stdout);
        }
        sleep_ms(100);
        watchdog_update();
        
        // Check for user abort
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            if (!quiet_mode) {
                printf("\nTest aborted by user\n");
            }
            break;
        }
    }
    
    // Clear motor override to stop
    debug_clear_motor_override();
    
    if (!quiet_mode) {
        printf("\nCustom motor test complete\n");
    }
}

void debug_test_encoder_direction(void) {
    if (!enc_found) {
        if (!quiet_mode) {
            printf("Error: No encoder found\n");
        }
        return;
    }
    
    if (!quiet_mode) {
        printf("\n=== Encoder Direction Test ===\n");
        printf("Manually move the pendulum slowly and observe the angle change.\n");
        printf("Moving towards positive (counterclockwise when viewed from encoder side)\n");
        printf("should show INCREASING angles.\n");
        printf("Press any key to stop...\n\n");
    }
    
    float last_angle = 0.0f;
    bool first_reading = true;
    
    while (true) {
        float angle_raw;
        if (as5600_read_angle_rad(&enc, &angle_raw)) {
            float theta_b = wrap_pi(angle_raw - enc.angle_offset_rad);
            
            if (!first_reading) {
                float delta = theta_b - last_angle;
                // Handle wrap-around
                if (delta > M_PI) delta -= 2.0f * M_PI;
                if (delta < -M_PI) delta += 2.0f * M_PI;
                
                char direction = ' ';
                if (fabsf(delta) > 0.001f) {  // Threshold to avoid noise
                    direction = (delta > 0) ? '+' : '-';
                }
                
                if (!quiet_mode) {
                    printf("Raw: %6.3f  |  Theta_b: %6.3f  |  Delta: %c%6.3f  |  Deg: %6.1f\r", 
                           angle_raw, theta_b, direction, fabsf(delta), theta_b * 180.0f / M_PI);
                    fflush(stdout);
                }
            }
            
            last_angle = theta_b;
            first_reading = false;
        }
        
        sleep_ms(100);
        
        // Check for user input to stop
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            break;
        }
    }
    
    if (!quiet_mode) {
        printf("\nEncoder direction test complete\n");
    }
}

void debug_test_motor_encoder_sign_match(void) {
    if (!enc_found) {
        if (!quiet_mode) {
            printf("Error: No encoder found\n");
        }
        return;
    }
    
    if (!quiet_mode) {
        printf("\n=== Motor-Encoder Sign Match Test ===\n");
        printf("This test applies positive motor command and observes encoder response.\n");
        printf("For correct operation:\n");
        printf("  - Positive motor command should produce positive angular velocity\n");
        printf("  - If signs are mismatched, use 'inv' command to toggle MOTOR_INVERT\n");
        printf("Current MOTOR_INVERT setting: %s\n", MOTOR_INVERT ? "TRUE" : "FALSE");
        printf("Starting test in 3 seconds...\n");
    }
    
    // Wait 3 seconds
    for (int i = 3; i > 0; i--) {
        if (!quiet_mode) {
            printf("%d...\n", i);
        }
        sleep_ms(1000);
        watchdog_update();
    }
    
    // Record starting position
    float start_angle = S.theta_b;
    
    if (!quiet_mode) {
        printf("Applying +30%% motor command for 2 seconds...\n");
        printf("Starting angle: %.3f rad (%.1f deg)\n", start_angle, start_angle * 180.0f / M_PI);
    }
    
    // Apply positive motor command
    debug_set_motor_override(0.3f, 3000);  // 30% for 3 seconds (safety margin)
    
    // Monitor for 2 seconds
    for (int i = 0; i < 20; i++) {
        float current_angle = S.theta_b;
        float angular_velocity = S.omega;
        
        if (!quiet_mode && i % 5 == 0) {  // Every 500ms
            printf("t=%.1fs: angle=%.3f, omega=%.3f rad/s\n", 
                   i * 0.1f, current_angle, angular_velocity);
        }
        
        sleep_ms(100);
        watchdog_update();
    }
    
    // Stop motor
    debug_clear_motor_override();
    
    // Wait a moment for settling
    sleep_ms(500);
    
    float end_angle = S.theta_b;
    float total_displacement = end_angle - start_angle;
    
    // Handle wrap-around
    if (total_displacement > M_PI) total_displacement -= 2.0f * M_PI;
    if (total_displacement < -M_PI) total_displacement += 2.0f * M_PI;
    
    if (!quiet_mode) {
        printf("\n=== Test Results ===\n");
        printf("Start angle: %.3f rad (%.1f deg)\n", start_angle, start_angle * 180.0f / M_PI);
        printf("End angle:   %.3f rad (%.1f deg)\n", end_angle, end_angle * 180.0f / M_PI);
        printf("Net displacement: %.3f rad (%.1f deg)\n", total_displacement, total_displacement * 180.0f / M_PI);
        printf("Motor command: +30%% (with MOTOR_INVERT=%s)\n", MOTOR_INVERT ? "TRUE" : "FALSE");
        
        if (fabsf(total_displacement) < 0.05f) {
            printf("RESULT: Little/no movement detected - check motor connections or increase power\n");
        } else if (total_displacement > 0.05f) {
            printf("RESULT: POSITIVE displacement - signs match correctly\n");
        } else {
            printf("RESULT: NEGATIVE displacement - consider toggling MOTOR_INVERT\n");
        }
        printf("Use 'inv' command to toggle motor inversion if needed\n");
    }
}

void debug_wake_pendulum(void) {
    if (!enc_found) {
        if (!quiet_mode) {
            printf("No encoder found - cannot start swing-up mode\n");
        }
        return;
    }
    
    if (!control_enabled) {
        if (!quiet_mode) {
            printf("Control is disabled - use 'm' to enable first\n");
        }
        return;
    }
    
    if (!quiet_mode) {
        printf("Starting swing-up mode...\n");
    }
    S.state = ST_SWINGUP;
    
    // Give a small perturbation to start the motion
    if (!quiet_mode) {
        printf("Applying initial perturbation...\n");
    }
    drv8833_cmd(&drv, 0.1f);  // Small forward pulse
    sleep_ms(100);
    watchdog_update();
    drv8833_cmd(&drv, -0.1f); // Small reverse pulse
    sleep_ms(100);
    watchdog_update();
    drv8833_cmd(&drv, 0.0f);  // Let control take over
    
    if (!quiet_mode) {
        printf("Swing-up mode active - pendulum should start moving\n");
    }
}

void debug_reset_thermal(void) {
    thermal_protection_active = false;
    if (!quiet_mode) {
        printf("Thermal protection reset - normal power limits restored\n");
    }
}

void debug_show_encoder_realtime(void) {
    if (!enc_found) {
        if (!quiet_mode) {
            printf("No encoder found! Press any key to exit.\n");
        }
        return;
    }
    
    // Temporarily disable control to avoid I2C conflicts
    bool was_enabled = control_enabled;
    if (was_enabled) {
        control_enabled = false;
        drv8833_cmd(&drv, 0.0f);  // Stop motor
        if (!quiet_mode) {
            printf("Control temporarily disabled for encoder monitoring\n");
        }
        sleep_ms(100);  // Let control loop finish
    }
    
    if (!quiet_mode) {
        printf("Encoder realtime monitor. Press any key to exit.\n");
        printf("Format: Raw | Angle(deg) | Status\n");
        printf("-------------------------------\n");
    }
    
    int max_iterations = 100;  // Safety limit: 10 seconds at 10Hz
    int iteration = 0;
    
    while (iteration < max_iterations) {
        iteration++;
        
        // Feed watchdog to prevent system reset
        watchdog_update();
        
        // Check for any key press to exit
        int ch = getchar_timeout_us(0);
        if (ch != PICO_ERROR_TIMEOUT) {
            if (!quiet_mode) {
                printf("\nExiting encoder monitor\n");
            }
            break;
        }
        
        // Use the existing AS5600 functions which have proper error handling
        uint16_t raw_angle;
        float angle_rad;
        bool angle_ok = as5600_read_raw(&enc, &raw_angle);
        bool rad_ok = as5600_read_angle_rad(&enc, &angle_rad);
        
        // Simple status check
        uint8_t status = 0;
        uint8_t status_reg = 0x0B;
        bool status_ok = (i2c_write_timeout_us(i2c1, 0x36, &status_reg, 1, true, 2000) == 1) &&
                         (i2c_read_timeout_us(i2c1, 0x36, &status, 1, false, 2000) == 1);
        
        // Display results
        if (angle_ok && rad_ok) {
            if (!quiet_mode) {
                printf("\r%4u | %6.1f° | 0x%02X %s%s%s        ", 
                       raw_angle,
                       angle_rad * 180.0f / 3.14159f,  // Convert to degrees
                       status,
                       status_ok ? ((status & 0x20) ? "MAG " : "    ") : "ERR ",
                       status_ok ? ((status & 0x10) ? "AGC " : "    ") : "",
                       status_ok ? ((status & 0x08) ? "WEAK" : "    ") : ""
                );
            }
        } else {
            if (!quiet_mode) {
                printf("\rEncoder read ERROR - check connections!        ");
            }
        }
        if (!quiet_mode) {
            fflush(stdout);
        }
        
        sleep_ms(100);  // Update 10 times per second
    }
    
    if (iteration >= max_iterations) {
        if (!quiet_mode) {
            printf("\nEncoder monitor timeout reached\n");
        }
    }
    
    // Re-enable control if it was previously enabled
    if (was_enabled) {
        if (!quiet_mode) {
            printf("Re-enabling control...\n");
        }
        sleep_ms(100);
        control_enabled = true;
    }
}

void debug_test_direction(void) {
    if (!quiet_mode) {
        printf("\n=== Encoder Direction & Motor Polarity Test ===\n");
        printf("This test will apply motor commands and read encoder changes\n");
        printf("to determine if motor leads are connected correctly.\n\n");
    }
    
    // Disable control temporarily
    bool was_enabled = control_enabled;
    if (was_enabled) {
        control_enabled = false;
        drv8833_cmd(&drv, 0.0f);
        if (!quiet_mode) {
            printf("Control temporarily disabled for direction test\n");
        }
        sleep_ms(100);
    }
    
    if (!enc_found) {
        if (!quiet_mode) {
            printf("ERROR: No encoder found! Cannot test direction.\n");
        }
        goto restore_control;
    }
    
    // Get initial encoder reading
    float angle1;
    if (!as5600_read_angle_rad(&enc, &angle1)) {
        if (!quiet_mode) {
            printf("ERROR: Cannot read encoder!\n");
        }
        goto restore_control;
    }
    
    if (!quiet_mode) {
        printf("Initial encoder angle: %.3f rad (%.1f deg)\n", angle1, angle1 * 180.0f / M_PI);
        printf("Applying positive motor command (+30%% power for 1 second)...\n");
    }
    
    // Apply positive motor command
    drv8833_cmd(&drv, 0.3f);
    sleep_ms(1000);
    drv8833_cmd(&drv, 0.0f);
    sleep_ms(200);  // Let it settle
    
    // Read encoder again
    float angle2;
    if (!as5600_read_angle_rad(&enc, &angle2)) {
        if (!quiet_mode) {
            printf("ERROR: Cannot read encoder after motor test!\n");
        }
        goto restore_control;
    }
    
    // Calculate angle change (handle wrap-around)
    float delta = angle2 - angle1;
    if (delta > M_PI) delta -= 2.0f * M_PI;
    if (delta < -M_PI) delta += 2.0f * M_PI;
    
    if (!quiet_mode) {
        printf("Final encoder angle: %.3f rad (%.1f deg)\n", angle2, angle2 * 180.0f / M_PI);
        printf("Angle change: %.3f rad (%.1f deg)\n", delta, delta * 180.0f / M_PI);
        
        if (fabsf(delta) < 0.05f) {  // Less than ~3 degrees
            printf("\nWARNING: Very small angle change detected!\n");
            printf("- Motor might be stuck or have insufficient power\n");
            printf("- Check mechanical coupling and friction\n");
            printf("- Try holding pendulum and running test again\n");
        } else {
            printf("\nDirection Analysis:\n");
            if (delta > 0) {
                printf("- Positive motor command caused CLOCKWISE rotation\n");
                printf("- This means positive control output moves pendulum CW\n");
            } else {
                printf("- Positive motor command caused COUNTER-CLOCKWISE rotation\n");
                printf("- This means positive control output moves pendulum CCW\n");
            }
            
            printf("\nFor proper swing-up operation:\n");
            printf("- The motor should push the pendulum in the direction that\n");
            printf("  increases energy when moving toward bottom position\n");
            printf("- If swing-up seems backwards, swap the motor leads\n");
            printf("- The encoder direction is automatically handled by the algorithm\n");
        }
        
        printf("\nTest complete. Motor leads appear to be: ");
        if (delta > 0) {
            printf("CORRECT for standard configuration\n");
        } else {
            printf("POSSIBLY REVERSED - try swapping if swing-up is ineffective\n");
        }
    }
    
restore_control:
    // Re-enable control if it was previously enabled
    if (was_enabled) {
        if (!quiet_mode) {
            printf("\nRe-enabling control...\n");
        }
        sleep_ms(100);
        control_enabled = true;
    }
}

void debug_recalibrate_down(void) {
    if (!quiet_mode) {
        printf("\n=== Recalibrate Down Position ===\n");
        printf("Hold the pendulum at the bottom hanging position.\n");
        printf("Calibration will start automatically in 5 seconds...\n");
    }
    
    if (!enc_found) {
        if (!quiet_mode) {
            printf("ERROR: No encoder found! Cannot calibrate.\n");
        }
        return;
    }
    
    // Disable control temporarily
    bool was_enabled = control_enabled;
    if (was_enabled) {
        control_enabled = false;
        drv8833_cmd(&drv, 0.0f);
        if (!quiet_mode) {
            printf("Control disabled for calibration.\n");
        }
        sleep_ms(100);
    }
    
    // Give user time to position pendulum - no serial input required
    for (int countdown = 5; countdown > 0; countdown--) {
        if (!quiet_mode) {
            printf("Starting in %d seconds...\n", countdown);
        }
        sleep_ms(1000);
        watchdog_update();  // Prevent watchdog reset during countdown
    }
    
    if (!quiet_mode) {
        printf("Starting calibration now...\n");
    }
    
    // Collect calibration samples
    float cal_sum = 0.0f;
    int cal_count = 0;
    int stable_count = 0;
    float last_reading = 0.0f;
    
    if (!quiet_mode) {
        printf("Collecting calibration data (hold pendulum steady at bottom)...\n");
    }
    
    for (int i = 0; i < 50; i++) {  // 5 second calibration
        watchdog_update();  // Prevent watchdog reset during calibration
        
        float angle_raw;
        if (as5600_read_angle_rad(&enc, &angle_raw)) {
            // Check stability
            if (cal_count > 0 && fabsf(angle_raw - last_reading) < 0.05f) {
                stable_count++;
            } else {
                stable_count = 0;
            }
            
            cal_sum += angle_raw;
            cal_count++;
            last_reading = angle_raw;
            
            if (i % 10 == 0 && !quiet_mode) {
                printf("Sample %d/50: %.3f rad (%.1f deg), stable: %d\n", 
                       i, angle_raw, angle_raw * 180.0f / M_PI, stable_count);
            }
            
            // Early exit if very stable
            if (stable_count >= 15 && cal_count >= 20) {
                if (!quiet_mode) {
                    printf("Stable reading achieved early.\n");
                }
                break;
            }
        } else {
            if (!quiet_mode) {
                printf("Reading failed at sample %d\n", i);
            }
        }
        
        sleep_ms(100);
    }
    
    if (cal_count > 5) {
        float old_offset = enc.angle_offset_rad;
        enc.angle_offset_rad = cal_sum / cal_count;
        
        if (!quiet_mode) {
            printf("\nCalibration complete!\n");
            printf("Old offset: %.6f rad (%.2f deg)\n", old_offset, old_offset * 180.0f / M_PI);
            printf("New offset: %.6f rad (%.2f deg)\n", enc.angle_offset_rad, enc.angle_offset_rad * 180.0f / M_PI);
            printf("Change: %.6f rad (%.2f deg)\n", 
                   enc.angle_offset_rad - old_offset, 
                   (enc.angle_offset_rad - old_offset) * 180.0f / M_PI);
            printf("Calibration used %d samples.\n", cal_count);
        }
    } else {
        if (!quiet_mode) {
            printf("ERROR: Insufficient calibration data! Keeping old offset.\n");
        }
    }
    
    // Re-enable control if it was previously enabled
    if (was_enabled) {
        if (!quiet_mode) {
            printf("Re-enabling control...\n");
        }
        sleep_ms(100);
        control_enabled = true;
    }
}

void debug_toggle_pwm_mode(void) {
    // Check current mode and toggle
    if (drv.control_mode == DRV8833_SIGN_MAGNITUDE) {
        drv8833_set_mode(&drv, DRV8833_LOCKED_ANTIPHASE);
        if (!quiet_mode) {
            printf("PWM mode set to LOCKED ANTI-PHASE\n");
            printf("  - Both pins PWM at complementary duty cycles\n");
            printf("  - Reduces motor cogging and improves smoothness\n");
            printf("  - Better for precision control at low speeds\n");
        }
    } else {
        drv8833_set_mode(&drv, DRV8833_SIGN_MAGNITUDE);
        if (!quiet_mode) {
            printf("PWM mode set to SIGN-MAGNITUDE\n");
            printf("  - One pin PWM, other pin low (traditional)\n");
            printf("  - Lower power consumption\n");
            printf("  - Better for high-power applications\n");
        }
    }
}
