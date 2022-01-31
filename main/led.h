#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void led_init(void);
void led_set_duty(uint8_t id, uint32_t duty);

#define LED1_ON() led_set_duty(0, 4095)
#define LED2_ON() led_set_duty(1, 4095)
#define LED3_ON() led_set_duty(2, 4095)

#define LED1_OFF() led_set_duty(0, 0)
#define LED2_OFF() led_set_duty(1, 0)
#define LED3_OFF() led_set_duty(2, 0)

#ifdef __cplusplus
}
#endif
