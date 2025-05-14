#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED1 GPIO_NUM_4
#define LED2 GPIO_NUM_5
#define LED3 GPIO_NUM_6
#define LED4 GPIO_NUM_7
#define BTN_COUNT GPIO_NUM_12
#define BTN_INC GPIO_NUM_13

int contador = 0;      
int incremento = 1;    
int ant_inc = 1;     
int ant_count = 1;   

void incrementa(int valor) {
    contador = (contador + valor) % 16;
}

void atualiza_leds(int valor) {
    gpio_set_level(LED1, (valor >> 0) & 1);
    gpio_set_level(LED2, (valor >> 1) & 1);
    gpio_set_level(LED3, (valor >> 2) & 1);
    gpio_set_level(LED4, (valor >> 3) & 1);
}

void app_main() {

    gpio_config_t io_conf_leds = {
        .pin_bit_mask = (1ULL << LED1) | (1ULL << LED2) | (1ULL << LED3) | (1ULL << LED4),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf_leds);

    gpio_config_t io_conf_btns = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_COUNT),
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = 1,   
        .pull_down_en = 0  
    };
    gpio_config(&io_conf_btns);

    while (1) {
        int inc = gpio_get_level(BTN_INC);   
        int count = gpio_get_level(BTN_COUNT); 
        if (inc == 0 && ant_inc == 1) { 
            incrementa(incremento);
            atualiza_leds(contador);
        }
        ant_inc = inc; 
        if (count == 0 && ant_count == 1) {  
            incremento = (incremento == 1) ? 2 : 1;
        }
        ant_count = count; 
        vTaskDelay(pdMS_TO_TICKS(50)); 
    };
}
