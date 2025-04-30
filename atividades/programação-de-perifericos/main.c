#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define led_1 0
#define led_2 1

void app_main(void) {
    gpio_set_direction(led_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(led_2, GPIO_MODE_OUTPUT);
    int count_200ms = 0;
    int count_1000ms = 0;
    int state_led1 = 0;
    int state_led2 = 0;
    while (1) {
        count_200ms += 100;
        count_1000ms += 100;
        if (count_200ms >= 200) {
            state_led1 = !state_led1;
            gpio_set_level(led_1, state_led1);
            count_200ms = 0;
        }
        if (count_1000ms >= 1000) {
            state_led2 = !state_led2;
            gpio_set_level(led_2, state_led2);
            count_1000ms = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
