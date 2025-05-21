#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"

#define LED1 GPIO_NUM_4
#define LED2 GPIO_NUM_5
#define LED3 GPIO_NUM_6
#define LED4 GPIO_NUM_7
#define BTN_INC GPIO_NUM_12
#define BTN_MOD GPIO_NUM_13 
volatile uint8_t flag_inc = 0;
volatile uint8_t flag_mod = 0;
volatile int incremento = 1;
int contador = 0;

void atualiza_leds(int valor) {
    gpio_set_level(LED1, (valor >> 0) & 1);
    gpio_set_level(LED2, (valor >> 1) & 1);
    gpio_set_level(LED3, (valor >> 2) & 1);
    gpio_set_level(LED4, (valor >> 3) & 1);
}

void IRAM_ATTR isr_incrementa(void *arg) {
    flag_inc = 1;
}

void IRAM_ATTR isr_modo(void *arg) {
    flag_mod = 1;
}

void app_main() {
    gpio_config_t conf_leds = {
        .pin_bit_mask = (1ULL << LED1) | (1ULL << LED2) |
                        (1ULL << LED3) | (1ULL << LED4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&conf_leds);

    gpio_config_t conf_botoes = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_MOD),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&conf_botoes);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_INC, isr_incrementa, NULL);
    gpio_isr_handler_add(BTN_MOD, isr_modo, NULL);

    atualiza_leds(contador);

    while (1) {
        if (flag_mod) {
            flag_mod = 0;
            incremento = (incremento == 1) ? 2 : 1;
        }

        if (flag_inc) {
            flag_inc = 0;
            contador = (contador + incremento) % 16;
            atualiza_leds(contador);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
