
#ifndef AS5600_H
#define AS5600_H

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5600_I2C_ADDR 0x36

typedef struct {
    i2c_inst_t *i2c;
    uint sda_gpio;
    uint scl_gpio;
    uint8_t addr;
    float angle_offset_rad; // offset to make bottom (down) = 0
    bool ok;
} as5600_t;

// Initialize I2C and sensor
bool as5600_init(as5600_t *dev, i2c_inst_t *i2c, uint sda_gpio, uint scl_gpio, uint baud);

// Read raw 12-bit angle (0..4095). Returns true on success.
bool as5600_read_raw(const as5600_t *dev, uint16_t *raw);

// Read angle in radians [0, 2π)
bool as5600_read_angle_rad(const as5600_t *dev, float *angle_rad);

// Read AGC and magnitude (diagnostic). Returns true on success.
bool as5600_read_agc_mag(const as5600_t *dev, uint8_t *agc, uint16_t *magnitude);

// Simple utility: unwrap wrapped angle to continuous angle (radians)
static inline float unwrap_angle(float prev, float now_wrapped) {
    // both in radians [0, 2π)
    const float TWO_PI = 6.283185307179586f;
    float diff = now_wrapped - prev;
    if (diff > 3.14159265f) now_wrapped -= TWO_PI;
    else if (diff < -3.14159265f) now_wrapped += TWO_PI;
    return now_wrapped;
}

#ifdef __cplusplus
}
#endif

#endif
