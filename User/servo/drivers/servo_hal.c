#include "servo_hal.h"

#include "tim.h"

#define SERVO0_TIM     (&htim2)
#define SERVO0_CHANNEL TIM_CHANNEL_2
#define SERVO1_TIM     (&htim4)
#define SERVO1_CHANNEL TIM_CHANNEL_3
#define SERVO2_TIM     (&htim4)
#define SERVO2_CHANNEL TIM_CHANNEL_4
#define SERVO3_TIM     (&htim4)
#define SERVO3_CHANNEL TIM_CHANNEL_1
#define SERVO4_TIM     (&htim4)
#define SERVO4_CHANNEL TIM_CHANNEL_2
#define SERVO5_TIM     (&htim3)
#define SERVO5_CHANNEL TIM_CHANNEL_1

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t           channel;
} servo_hw_t;

static servo_hw_t servo_hw_map[] = {
    {SERVO0_TIM, SERVO0_CHANNEL}, // servo 0
    {SERVO1_TIM, SERVO1_CHANNEL}, // servo 1
    {SERVO2_TIM, SERVO2_CHANNEL}, // servo 2
    {SERVO3_TIM, SERVO3_CHANNEL}, // servo 3
    {SERVO4_TIM, SERVO4_CHANNEL}, // servo 4
    {SERVO5_TIM, SERVO5_CHANNEL}, // servo 5
};

#define SERVO_NUM (sizeof(servo_hw_map) / sizeof(servo_hw_map[0]))

void servo_hal_init(void)
{
    for (unsigned int i = 0; i < SERVO_NUM; i++) {
        HAL_TIM_PWM_Start(servo_hw_map[i].htim, servo_hw_map[i].channel);
    }
}

void servo_hal_set_pwm(uint32_t servo_id, uint32_t pwm_us)
{
    if (servo_id >= SERVO_NUM) return;

    TIM_HandleTypeDef* htim = servo_hw_map[servo_id].htim;
    uint32_t           ch   = servo_hw_map[servo_id].channel;

    __HAL_TIM_SET_COMPARE(htim, ch, pwm_us);
}
