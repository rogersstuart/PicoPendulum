#include "embedded/motor_protection.h"
#include <stdio.h>

int main() {
    printf("MOTOR_THERMAL_CAPACITY = %.1f\n", MOTOR_THERMAL_CAPACITY);
    printf("MOTOR_EMERGENCY_CAPACITY = %.1f\n", MOTOR_EMERGENCY_CAPACITY);
    return 0;
}
