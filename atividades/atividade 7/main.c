#include <stdio.h>
#include <stdarg.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_rom_sys.h"
#include <math.h>

#define LED1 GPIO_NUM_4
#define LED2 GPIO_NUM_5
#define LED3 GPIO_NUM_6
#define LED4 GPIO_NUM_7
#define BOTAO_INC GPIO_NUM_12
#define BOTAO_DEC GPIO_NUM_13
#define BUZZER GPIO_NUM_10
#define NTC_ADC_CHANNEL ADC1_CHANNEL_0

#define SDA_PIN 9
#define SCL_PIN 8

#define RS 0
#define RW 1
#define EN 2
#define BL 3
#define DISPLAY_16X02 0

#define CLEAR_DISPLAY               0x01
#define SET_4BIT_MODE               0x28
#define DISPLAY_ON_CURSOR_OFF       0x0C
#define CURSOR_RIGHT_NO_SHIFT_LEFT  0x06

#define NTC_SERIES_RESISTOR     10000
#define NTC_BETA                3950
#define NTC_NOMINAL_RESISTANCE  10000
#define NTC_NOMINAL_TEMPERATURE 25.0

#define NUM_AMOSTRAS_FILTRO 10
#define INTERVALO_LEITURA_MS 50

volatile uint32_t ultima_vez_inc = 0;
volatile uint32_t ultima_vez_dec = 0;
const TickType_t debounce_ticks = pdMS_TO_TICKS(200);
volatile float temperatura_alarme = 25.0;
QueueHandle_t fila_botoes;

typedef enum {
    INCREMENTA,
    DECREMENTA
} tipo_botao_t;

typedef struct {
    uint8_t address;
    uint8_t num;
    uint8_t backlight;
    uint8_t size;
} lcd_i2c_handle_t;

lcd_i2c_handle_t display = {
    .address = 0x27,
    .num = I2C_NUM_1,
    .backlight = 1,
    .size = DISPLAY_16X02
};

void configurar_leds() {
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);
}

void configurar_pwm() {
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t canal = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&canal);
}

void configurar_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = SCL_PIN,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_1, &conf);
    i2c_driver_install(I2C_NUM_1, conf.mode, 0, 0, 0);
}

void i2c_write_byte(lcd_i2c_handle_t *lcd, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lcd->address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(lcd->num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void lcd_send_nibble(lcd_i2c_handle_t *lcd, uint8_t nibble, uint8_t control) {
    uint8_t data = (nibble & 0xF0) | control | (lcd->backlight ? (1 << BL) : 0);
    i2c_write_byte(lcd, data | (1 << EN));
    esp_rom_delay_us(500);
    i2c_write_byte(lcd, data & ~(1 << EN));
    esp_rom_delay_us(500);
}

void lcd_i2c_write(lcd_i2c_handle_t *lcd, char rs_flag, uint8_t data_byte) {
    uint8_t control = rs_flag ? (1 << RS) : 0;
    lcd_send_nibble(lcd, data_byte & 0xF0, control);
    lcd_send_nibble(lcd, (data_byte << 4) & 0xF0, control);
}

void lcd_i2c_init(lcd_i2c_handle_t *lcd) {
    vTaskDelay(50 / portTICK_PERIOD_MS);
    lcd_send_nibble(lcd, 0x30, 0); vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_send_nibble(lcd, 0x30, 0); vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_send_nibble(lcd, 0x20, 0);
    lcd_i2c_write(lcd, 0, SET_4BIT_MODE);
    lcd_i2c_write(lcd, 0, DISPLAY_ON_CURSOR_OFF);
    lcd_i2c_write(lcd, 0, CLEAR_DISPLAY);
    lcd_i2c_write(lcd, 0, CURSOR_RIGHT_NO_SHIFT_LEFT);
}

void lcd_i2c_cursor_set(lcd_i2c_handle_t *lcd, uint8_t col, uint8_t row) {
    uint8_t row_offsets[] = {0x00, 0x40};
    lcd_i2c_write(lcd, 0, 0x80 + row_offsets[row] + col);
}

void lcd_i2c_print(lcd_i2c_handle_t *lcd, const char *fmt, ...) {
    char buffer[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    for (int i = 0; buffer[i] != '\0'; i++) {
        lcd_i2c_write(lcd, 1, buffer[i]);
    }
}

float ler_temp_unica() {
    int leitura = adc1_get_raw(NTC_ADC_CHANNEL);
    if (leitura <= 0 || leitura >= 4095) {
        return NAN; 
    }
    float resistencia = (float)NTC_SERIES_RESISTOR * leitura / (4095.0 - leitura);
    float tempK = 1.0f / (1.0f / (NTC_NOMINAL_TEMPERATURE + 273.15f)
                      + log(resistencia / NTC_NOMINAL_RESISTANCE) / NTC_BETA);
    float tempC = tempK - 273.15f;
    return tempC;
}

void atualiza_display(float temp, float alarme) {
    lcd_i2c_cursor_set(&display, 0, 0);
    if (isnan(temp)) {
        lcd_i2c_print(&display, "Temp: ---.- C   ");
        lcd_i2c_print(&display, "Temp: %.1f C   ", temp); 
    }
    lcd_i2c_cursor_set(&display, 0, 1);
    lcd_i2c_print(&display, "Alarme: %.1f C ", alarme);
}

void atualizar_leds(float temp, float alarme) {
    float diferenca = fabs(alarme - temp); 
    if (temp >= alarme) {
        gpio_set_level(LED1, 1);
        gpio_set_level(LED2, 1);
        gpio_set_level(LED3, 1);
        gpio_set_level(LED4, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
        gpio_set_level(LED1, 0);
        gpio_set_level(LED2, 0);
        gpio_set_level(LED3, 0);
        gpio_set_level(LED4, 0);
        vTaskDelay(pdMS_TO_TICKS(150));
    } else {
        gpio_set_level(LED1, 0);
        gpio_set_level(LED2, 0);
        gpio_set_level(LED3, 0);
        gpio_set_level(LED4, 0);
        if (diferenca <= 2.0f) {
            gpio_set_level(LED1, 1);
            gpio_set_level(LED2, 1);
            gpio_set_level(LED3, 1);
            gpio_set_level(LED4, 1);
        } else if (diferenca <= 10.0f) {
            gpio_set_level(LED1, 1);
            gpio_set_level(LED2, 1);
            gpio_set_level(LED3, 1);
        } else if (diferenca <= 15.0f) {
            gpio_set_level(LED1, 1);
            gpio_set_level(LED2, 1);
        } else if (diferenca <= 20.0f) {
            gpio_set_level(LED1, 1);
        }
    }
}

void IRAM_ATTR isr_botao_inc() {
    uint32_t agora = xTaskGetTickCountFromISR();
    if (agora - ultima_vez_inc > debounce_ticks) {
        ultima_vez_inc = agora;
        tipo_botao_t evento = INCREMENTA;
        xQueueSendFromISR(fila_botoes, &evento, NULL);
    }
}

void IRAM_ATTR isr_botao_dec() {
    uint32_t agora = xTaskGetTickCountFromISR();
    if (agora - ultima_vez_dec > debounce_ticks) {
        ultima_vez_dec = agora;
        tipo_botao_t evento = DECREMENTA;
        xQueueSendFromISR(fila_botoes, &evento, NULL);
    }
}

void task_botao(void *param) {
    tipo_botao_t evento;
    while (1) {
        if (xQueueReceive(fila_botoes, &evento, portMAX_DELAY)) {
            if (evento == INCREMENTA) {
                if (temperatura_alarme < 100.0f) {
                    temperatura_alarme += 5.0f;
                }
            } else if (evento == DECREMENTA) {
                if (temperatura_alarme > -20.0f) {
                    temperatura_alarme -= 5.0f;
                }
            }
        }
    }
}

void atualizar_buzzer(float temp, float alarme) {
    if (temp >= alarme) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void app_main(void) {

    gpio_set_direction(BOTAO_INC, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOTAO_INC, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(BOTAO_INC, GPIO_INTR_NEGEDGE);
    gpio_set_direction(BOTAO_DEC, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOTAO_DEC, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(BOTAO_DEC, GPIO_INTR_NEGEDGE);

    i2c_init();
    lcd_i2c_init(&display);
    configurar_leds();
    configurar_pwm();
    configurar_adc();

    fila_botoes = xQueueCreate(10, sizeof(tipo_botao_t));
    xTaskCreate(task_botao, "task_botao", 2048, NULL, 10, NULL);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOTAO_INC, isr_botao_inc, NULL);
    gpio_isr_handler_add(BOTAO_DEC, isr_botao_dec, NULL);

    while (1) {
        float temp_unica = ler_temp_unica();
        atualiza_display(temp_unica, temperatura_alarme);
        atualizar_leds(temp_unica, temperatura_alarme);
        atualizar_buzzer(temp_unica, temperatura_alarme);
        vTaskDelay(pdMS_TO_TICKS(INTERVALO_LEITURA_MS));
    }
}
