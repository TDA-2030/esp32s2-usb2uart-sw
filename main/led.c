#include <stdio.h>
#include "driver/ledc.h"
#include "hal/ledc_ll.h"
#include "esp_err.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES           LEDC_TIMER_12_BIT
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz


void led_init(void)
{
    // Set the LEDC peripheral configuration
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = CONFIG_BRIDGE_GPIO_LED1,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));


    ledc_channel.channel        = LEDC_CHANNEL_1;
    ledc_channel.gpio_num       = CONFIG_BRIDGE_GPIO_LED2;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_channel.channel        = LEDC_CHANNEL_2;
    ledc_channel.gpio_num       = CONFIG_BRIDGE_GPIO_LED3;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI("led", "LED init done");
}

void led_set_duty(uint8_t id, uint32_t duty)
{
    // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, id, duty));
    // // Update duty to apply the new value
    // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, id));

    ledc_ll_set_duty_int_part(LEDC_LL_GET_HW(), LEDC_MODE, id, duty);
    ledc_ll_ls_channel_update(LEDC_LL_GET_HW(), LEDC_MODE, id);
    ledc_ll_set_duty_start(LEDC_LL_GET_HW(), LEDC_MODE, id, 1);
}
