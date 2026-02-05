#ifndef SERVO_HAL_H
#define SERVO_HAL_H

#include <stdint.h>

void servo_hal_init(void);

/**
 * @brief 输出 PWM（单位 us）
 */
void servo_hal_set_pwm(uint32_t servo_id, uint32_t pwm_us);

#endif
