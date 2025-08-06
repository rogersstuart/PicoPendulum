
#include "as5600.h"
#include <stdio.h>

static bool as5600_read_regs(const as5600_t *dev, uint8_t reg, uint8_t *buf, size_t n) {
    int rv = i2c_write_blocking(dev->i2c, dev->addr, &reg, 1, true);
    if (rv != 1) return false;
    rv = i2c_read_blocking(dev->i2c, dev->addr, buf, n, false);
    return rv == (int)n;
}

bool as5600_init(as5600_t *dev, i2c_inst_t *i2c, uint sda_gpio, uint scl_gpio, uint baud) {
    dev->i2c = i2c;
    dev->sda_gpio = sda_gpio;
    dev->scl_gpio = scl_gpio;
    dev->addr = AS5600_I2C_ADDR;
    dev->angle_offset_rad = 0.0f;
    dev->ok = false;

    // Don't reinitialize I2C - assume it's already configured by main()
    // This prevents overriding the careful setup done in main.c
    
    // Test basic communication first (silently)
    uint8_t test_reg = 0x0B; // STATUS register
    uint8_t status;
    int result = i2c_write_blocking(i2c, AS5600_I2C_ADDR, &test_reg, 1, true);
    if (result != 1) {
        return false;
    }
    
    result = i2c_read_blocking(i2c, AS5600_I2C_ADDR, &status, 1, false);
    if (result != 1) {
        return false;
    }
    
    // Now try to read angle
    uint16_t raw;
    dev->ok = as5600_read_raw(dev, &raw);
    
    return dev->ok;
}

bool as5600_read_raw(const as5600_t *dev, uint16_t *raw) {
    uint8_t buf[2];
    // ANGLE register high/low: 0x0E, 0x0F (12-bit)
    if (!as5600_read_regs(dev, 0x0E, buf, 2)) return false;
    uint16_t v = ((uint16_t)buf[0] << 8) | buf[1];
    v &= 0x0FFF;
    *raw = v;
    return true;
}

bool as5600_read_angle_rad(const as5600_t *dev, float *angle_rad) {
    uint16_t raw;
    if (!as5600_read_raw(dev, &raw)) return false;
    const float TWO_PI = 6.283185307179586f;
    *angle_rad = (raw * (TWO_PI / 4096.0f));
    return true;
}

bool as5600_read_agc_mag(const as5600_t *dev, uint8_t *agc, uint16_t *magnitude) {
    uint8_t buf[3];
    // AGC: 0x1A, MAGNITUDE: 0x1B (hi), 0x1C (lo)
    if (!as5600_read_regs(dev, 0x1A, buf, 3)) return false;
    if (agc) *agc = buf[0];
    if (magnitude) *magnitude = ((uint16_t)buf[1] << 8) | buf[2];
    return true;
}
