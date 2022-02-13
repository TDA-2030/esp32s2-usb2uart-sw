#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void led_init(void);
void led_set_duty(uint8_t id, uint32_t duty);

#define LED1_ON() led_set_duty(0, 100)
#define LED2_ON() led_set_duty(1, 100)
#define LED3_ON() led_set_duty(2, 100)

#define LED1_OFF() led_set_duty(0, 0)
#define LED2_OFF() led_set_duty(1, 0)
#define LED3_OFF() led_set_duty(2, 0)

#define LED_JTAG_OFF LED3_OFF
#define LED_JTAG_ON LED3_ON

#ifdef __cplusplus
}
#endif
