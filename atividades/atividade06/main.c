#include <stdio.h>
#include <stdarg.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#define LED1 GPIO_NUM_4
#define LED2 GPIO_NUM_5
#define LED3 GPIO_NUM_6
#define LED4 GPIO_NUM_7
#define BOTAO_INCREMENTA GPIO_NUM_12
#define BOTAO_TROCA_INCREMENTO GPIO_NUM_13
#define LED_PWM GPIO_NUM_10

#define SDA_PIN 9
#define SCL_PIN 8

#define RS 0
#define RW 1
#define EN 2
#define BL 3
#define DISPLAY_16X02 0

#define CLEAR_DISPLAY               0x01
#define RETURN_HOME_UNSHIFT         0x02
#define SET_4BIT_MODE               0x28
#define DISPLAY_ON_CURSOR_OFF       0x0C
#define CURSOR_RIGHT_NO_SHIFT_LEFT  0x06

int contador = 0;
int incremento = 1;

typedef struct {
    uint8_t address;
    uint8_t num;
    uint8_t backlight;
    uint8_t size;
} lcd_i2c_handle_t;

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
        .gpio_num = LED_PWM,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&canal);
}

void atualizar_brilho(int valor) {
    int brilho = (valor * 255) / 15;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, brilho);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

void atualizar_leds(int valor) {
    gpio_set_level(LED1, (valor >> 0) & 1);
    gpio_set_level(LED2, (valor >> 1) & 1);
    gpio_set_level(LED3, (valor >> 2) & 1);
    gpio_set_level(LED4, (valor >> 3) & 1);
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

    if (data_byte == CLEAR_DISPLAY) {
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}

void lcd_i2c_init(lcd_i2c_handle_t *lcd) {
    vTaskDelay(50 / portTICK_PERIOD_MS);

    lcd_send_nibble(lcd, 0x30, 0);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_send_nibble(lcd, 0x30, 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);
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

lcd_i2c_handle_t display = {
    .address = 0x27,
    .num = I2C_NUM_1,
    .backlight = 1,
    .size = DISPLAY_16X02
};

void atualiza_display(int valor) {
    lcd_i2c_write(&display, 0, CLEAR_DISPLAY);
    lcd_i2c_cursor_set(&display, 0, 0);
    lcd_i2c_print(&display, "Contador:");
    lcd_i2c_cursor_set(&display, 0, 1);
    lcd_i2c_print(&display, "%d", valor);
}

void app_main(void) {
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);

    gpio_set_direction(BOTAO_INCREMENTA, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOTAO_INCREMENTA, GPIO_PULLUP_ONLY);
    gpio_set_direction(BOTAO_TROCA_INCREMENTO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOTAO_TROCA_INCREMENTO, GPIO_PULLUP_ONLY);

    i2c_init();
    lcd_i2c_init(&display);
    configurar_pwm();
    atualizar_leds(contador);
    atualizar_brilho(contador);
    atualiza_display(contador);

    while (1) {
        if (gpio_get_level(BOTAO_INCREMENTA) == 0) {
            contador = (contador + incremento) % 16;
            atualizar_leds(contador);
            atualizar_brilho(contador);
            atualiza_display(contador);
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }

        if (gpio_get_level(BOTAO_TROCA_INCREMENTO) == 0) {
            incremento = (incremento == 1) ? 2 : 1;
            vTaskDelay(300 / portTICK_PERIOD_MS);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
