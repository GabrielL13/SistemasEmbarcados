#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/adc.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "driver/sdspi_host.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "driver/sdmmc_types.h"
#include "sdkconfig.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define LED1 GPIO_NUM_4
#define LED2 GPIO_NUM_5
#define LED3 GPIO_NUM_6
#define LED4 GPIO_NUM_7
#define BOTAO_INC GPIO_NUM_12
#define BOTAO_DEC GPIO_NUM_13
#define BUZZER GPIO_NUM_10
#define NTC_ADC_CHANNEL ADC1_CHANNEL_0 

#define SDA_PIN GPIO_NUM_9
#define SCL_PIN GPIO_NUM_8
#define I2C_MASTER_NUM I2C_NUM_1
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_LEN 0
#define I2C_MASTER_RX_BUF_LEN 0
#define LCD_ADDRESS 0x27

#define RS 0
#define EN 2
#define BL 3

#define CLEAR_DISPLAY 0x01
#define SET_4BIT_MODE 0x28
#define DISPLAY_ON_CURSOR_OFF 0x0C
#define CURSOR_RIGHT_NO_SHIFT_LEFT 0x06

#define NTC_SERIES_RESISTOR 10000.0f
#define NTC_BETA 3950.0f
#define NTC_NOMINAL_RESISTANCE 10000.0f
#define NTC_NOMINAL_TEMPERATURE 25.0f
#define NUM_AMOSTRAS_NTC 10
#define INTERVALO_LEITURA_MS 1000
#define AJUSTE_TIMEOUT_MS 3000

#define PIN_NUM_MISO GPIO_NUM_17
#define PIN_NUM_MOSI GPIO_NUM_16
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_15

#define MOUNT_POINT "/sdcard"
#define LOG_FILE_PATH MOUNT_POINT "/log_temp.txt"

static const char *APP_TAG = "MONITOR_TEMP_SD";
static const char *LCD_TAG = "LCD_I2C";
static const char *SD_TAG = "SD_CARD";

volatile uint32_t ultima_vez_inc = 0;
volatile uint32_t ultima_vez_dec = 0;
const TickType_t debounce_ticks = pdMS_TO_TICKS(200);
volatile float temperatura_alarme = 25.0f;
QueueHandle_t fila_botoes;
volatile TickType_t last_adjustment_time;

typedef enum {
    ESTADO_INICIALIZANDO,
    ESTADO_MONITORANDO,
    ESTADO_AJUSTANDO_ALARME,
    ESTADO_ALARME_ATIVO
} estado_sistema_t;

volatile estado_sistema_t estado_atual = ESTADO_INICIALIZANDO;

typedef enum {
    INCREMENTA,
    DECREMENTA
} tipo_botao_t;

typedef struct {
    uint8_t address;
    i2c_port_t num;
    uint8_t backlight;
    uint8_t size;
} lcd_i2c_handle_t;

lcd_i2c_handle_t display = {
    .address = LCD_ADDRESS,
    .num = I2C_MASTER_NUM,
    .backlight = 1,
    .size = 0
};

sdmmc_card_t *card;
static sdmmc_host_t host = SDSPI_HOST_DEFAULT();

static esp_err_t lcd_send_cmd(lcd_i2c_handle_t *handle, uint8_t cmd) {
    uint8_t data_u, data_l;
    uint8_t data[4];
    data_u = (cmd & 0xF0);
    data_l = ((cmd << 4) & 0xF0);

    data[0] = data_u | (1 << EN) | (handle->backlight << BL);
    data[1] = data_u | (0 << EN) | (handle->backlight << BL);
    data[2] = data_l | (1 << EN) | (handle->backlight << BL);
    data[3] = data_l | (0 << EN) | (handle->backlight << BL);

    return i2c_master_write_to_device(handle->num, handle->address, data, 4, 1000 / portTICK_PERIOD_MS);
}

static esp_err_t lcd_send_data(lcd_i2c_handle_t *handle, uint8_t data_byte) {
    uint8_t data_u, data_l;
    uint8_t data[4];
    data_u = (data_byte & 0xF0);
    data_l = ((data_byte << 4) & 0xF0);

    data[0] = data_u | (1 << RS) | (1 << EN) | (handle->backlight << BL);
    data[1] = data_u | (1 << RS) | (0 << EN) | (handle->backlight << BL);
    data[2] = data_l | (1 << RS) | (1 << EN) | (handle->backlight << BL);
    data[3] = data_l | (1 << RS) | (0 << EN) | (handle->backlight << BL);

    return i2c_master_write_to_device(handle->num, handle->address, data, 4, 1000 / portTICK_PERIOD_MS);
}

void lcd_i2c_init(lcd_i2c_handle_t *handle) {
    esp_rom_delay_us(50000);

    lcd_send_cmd(handle, 0x30);
    esp_rom_delay_us(5000);
    lcd_send_cmd(handle, 0x30);
    esp_rom_delay_us(150);
    lcd_send_cmd(handle, 0x30);
    esp_rom_delay_us(150);

    lcd_send_cmd(handle, SET_4BIT_MODE);
    lcd_send_cmd(handle, SET_4BIT_MODE);
    lcd_send_cmd(handle, DISPLAY_ON_CURSOR_OFF);
    lcd_send_cmd(handle, CLEAR_DISPLAY);
    esp_rom_delay_us(2000);
    lcd_send_cmd(handle, CURSOR_RIGHT_NO_SHIFT_LEFT);

    ESP_LOGI(LCD_TAG, "LCD I2C inicializado com sucesso.");
}

void lcd_send_string(lcd_i2c_handle_t *handle, const char *str) {
    while (*str) {
        lcd_send_data(handle, (uint8_t)*str++);
    }
}

void lcd_set_cursor(lcd_i2c_handle_t *handle, uint8_t col, uint8_t row) {
    uint8_t address[] = {0x80, 0xC0, 0x94, 0xD4};
    if (row < 4) {
        lcd_send_cmd(handle, address[row] + col);
    }
}

void lcd_clear(lcd_i2c_handle_t *handle) {
    lcd_send_cmd(handle, CLEAR_DISPLAY);
    esp_rom_delay_us(2000);
}

void configurar_leds() {
    gpio_reset_pin(LED1);
    gpio_reset_pin(LED2);
    gpio_reset_pin(LED3);
    gpio_reset_pin(LED4);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);
    ESP_LOGI(APP_TAG, "LEDs configurados.");
}

void configurar_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = BUZZER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ESP_LOGI(APP_TAG, "Buzzer PWM configurado.");
}

void configurar_adc() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(NTC_ADC_CHANNEL, ADC_ATTEN_DB_11);
    ESP_LOGI(APP_TAG, "ADC configurado para NTC.");
}

void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_LEN, I2C_MASTER_TX_BUF_LEN, 0));
    ESP_LOGI(APP_TAG, "I2C configurado.");
}

static void IRAM_ATTR botao_inc_isr_handler(void *arg) {
    uint32_t agora = xTaskGetTickCountFromISR();
    if (agora - ultima_vez_inc > debounce_ticks) {
        tipo_botao_t botao = INCREMENTA;
        xQueueSendFromISR(fila_botoes, &botao, NULL);
        ultima_vez_inc = agora;
    }
}

static void IRAM_ATTR botao_dec_isr_handler(void *arg) {
    uint32_t agora = xTaskGetTickCountFromISR();
    if (agora - ultima_vez_dec > debounce_ticks) {
        tipo_botao_t botao = DECREMENTA;
        xQueueSendFromISR(fila_botoes, &botao, NULL);
        ultima_vez_dec = agora;
    }
}

void configurar_botoes() {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .pin_bit_mask = (1ULL << BOTAO_INC) | (1ULL << BOTAO_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);
    gpio_isr_handler_add(BOTAO_INC, botao_inc_isr_handler, NULL);
    gpio_isr_handler_add(BOTAO_DEC, botao_dec_isr_handler, NULL);
    ESP_LOGI(APP_TAG, "Botões configurados.");
}

void init_sd() {
    ESP_LOGI(SD_TAG, "Inicializando SD card...");

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    // FIX: Changed SPI_DMA_CHAN_NONE to SPI_DMA_CH_AUTO for compatibility with newer ESP-IDF versions
    ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO));

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(SD_TAG, "Falha ao montar o sistema de arquivos. Você tem certeza que o cartão está formatado como FAT?");
        } else {
            ESP_LOGE(SD_TAG, "Falha ao inicializar o cartão (%s). O pino CS está correto?", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(SD_TAG, "SD card montado com sucesso.");

    sdmmc_card_print_info(stdout, card);
}

float ler_temp_filtrada() {
    uint32_t adc_reading = 0;
    for (int i = 0; i < NUM_AMOSTRAS_NTC; i++) {
        adc_reading += adc1_get_raw(NTC_ADC_CHANNEL);
        esp_rom_delay_us(100);
    }
    adc_reading /= NUM_AMOSTRAS_NTC;

    float voltage = adc_reading * (3.9f / 4095.0f);

    float r_ntc = (NTC_SERIES_RESISTOR * voltage) / (3.3f - voltage);

    float temperatura_kelvin = 1.0f / ( (1.0f / (NTC_NOMINAL_TEMPERATURE + 273.15f)) + (1.0f / NTC_BETA) * logf(r_ntc / NTC_NOMINAL_RESISTANCE) );
    float temperatura_celsius = temperatura_kelvin - 273.15f;

    if (temperatura_celsius > 100.0f) temperatura_celsius = 100.0f;
    if (temperatura_celsius < -50.0f) temperatura_celsius = -50.0f;

    return temperatura_celsius;
}

void atualizar_leds(float temp, float alarme, estado_sistema_t estado) {
    if (estado == ESTADO_ALARME_ATIVO) {
        static bool piscar_estado = false;
        piscar_estado = !piscar_estado;
        gpio_set_level(LED1, piscar_estado);
        gpio_set_level(LED2, piscar_estado);
        gpio_set_level(LED3, piscar_estado);
        gpio_set_level(LED4, piscar_estado);
    } else {
        float diferenca = fabs(alarme - temp);
        gpio_set_level(LED1, 0);
        gpio_set_level(LED2, 0);
        gpio_set_level(LED3, 0);
        gpio_set_level(LED4, 0);

        if (diferenca <= 2.0f) {
            gpio_set_level(LED1, 1);
            gpio_set_level(LED2, 1);
            gpio_set_level(LED3, 1);
            gpio_set_level(LED4, 1);
        } else if (diferenca <= 5.0f) {
            gpio_set_level(LED1, 1);
            gpio_set_level(LED2, 1);
            gpio_set_level(LED3, 1);
        } else if (diferenca <= 10.0f) {
            gpio_set_level(LED1, 1);
            gpio_set_level(LED2, 1);
        } else if (diferenca <= 20.0f) {
            gpio_set_level(LED1, 1);
        }
    }
}

void controlar_buzzer(float temp, float alarme, estado_sistema_t estado) {
    if (estado == ESTADO_ALARME_ATIVO) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
}

void atualiza_display(float temp, float alarme) {
    char linha1[20];
    char linha2[20];

    lcd_clear(&display);

    snprintf(linha1, sizeof(linha1), "Temp: %.1f C", temp);
    snprintf(linha2, sizeof(linha2), "Alarme: %.1f C", alarme);

    lcd_set_cursor(&display, 0, 0);
    lcd_send_string(&display, linha1);

    lcd_set_cursor(&display, 0, 1);
    lcd_send_string(&display, linha2);
}

// FIX: Added const volatile to the pointer types to match the arguments passed from main task
void write_file(const float *temp_atual, const volatile float *temp_alarme) {
    FILE *f = fopen(LOG_FILE_PATH, "a");
    if (f == NULL) {
        ESP_LOGI(APP_TAG, "Temperatura: %.1f C, Alarme: %.1f C", *temp_atual, *temp_alarme);
        return;
    }

    TickType_t current_ticks = xTaskGetTickCount();
    float time_in_seconds = (float)current_ticks * portTICK_PERIOD_MS / 1000.0f;

    fprintf(f, "Tempo: %.2f s, Temp: %.1f C, Alarme: %.1f C, Estado: %d\n",
            time_in_seconds, *temp_atual, *temp_alarme, estado_atual);
    fclose(f);
}

void tarefa_controle_botoes(void *pvParameters) {
    tipo_botao_t botao;
    while (1) {
        if (xQueueReceive(fila_botoes, &botao, portMAX_DELAY)) {
            ESP_LOGI(APP_TAG, "Botão acionado. Transição para ESTADO_AJUSTANDO_ALARME.");
            estado_atual = ESTADO_AJUSTANDO_ALARME;
            last_adjustment_time = xTaskGetTickCount();

            if (botao == INCREMENTA) {
                temperatura_alarme += 5.0f;
                if (temperatura_alarme > 100.0f) temperatura_alarme = 100.0f;
                ESP_LOGI(APP_TAG, "Alarme incrementado para %.1f C", temperatura_alarme);
            } else if (botao == DECREMENTA) {
                temperatura_alarme -= 5.0f;
                if (temperatura_alarme < -50.0f) temperatura_alarme = -50.0f;
                ESP_LOGI(APP_TAG, "Alarme decrementado para %.1f C", temperatura_alarme);
            }
        }
    }
}

void tarefa_principal(void *pvParameters) {
    float temperatura_atual;
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = INTERVALO_LEITURA_MS / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        temperatura_atual = ler_temp_filtrada();

        switch (estado_atual) {
            case ESTADO_INICIALIZANDO:
                ESP_LOGI(APP_TAG, "Sistema no estado: INICIALIZANDO.");
                estado_atual = ESTADO_MONITORANDO;
                ESP_LOGI(APP_TAG, "Transicionando para ESTADO_MONITORANDO.");
                break;

            case ESTADO_MONITORANDO:
                atualizar_leds(temperatura_atual, temperatura_alarme, estado_atual);
                controlar_buzzer(temperatura_atual, temperatura_alarme, estado_atual);
                atualiza_display(temperatura_atual, temperatura_alarme);
                write_file(&temperatura_atual, &temperatura_alarme);
                if (temperatura_atual >= temperatura_alarme) {
                    ESP_LOGW(APP_TAG, "Temp (%.1f C) >= Alarme (%.1f C). Transicionando para ALARME_ATIVO.", temperatura_atual, temperatura_alarme);
                    estado_atual = ESTADO_ALARME_ATIVO;
                }
                break;

            case ESTADO_AJUSTANDO_ALARME:
                atualiza_display(temperatura_atual, temperatura_alarme);
                gpio_set_level(LED1, 0);
                gpio_set_level(LED2, 0);
                gpio_set_level(LED3, 0);
                gpio_set_level(LED4, 0);
                ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
                ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

                if ((xTaskGetTickCount() - last_adjustment_time) * portTICK_PERIOD_MS >= AJUSTE_TIMEOUT_MS) {
                    ESP_LOGI(APP_TAG, "Timeout em ajuste. Reavaliando estado...");
                    if (temperatura_atual >= temperatura_alarme) {
                        estado_atual = ESTADO_ALARME_ATIVO;
                        ESP_LOGW(APP_TAG, "Ajuste concluído. Temp (%.1f C) >= Alarme (%.1f C). Transicionando para ALARME_ATIVO.", temperatura_atual, temperatura_alarme);
                    } else {
                        estado_atual = ESTADO_MONITORANDO;
                        ESP_LOGI(APP_TAG, "Ajuste concluído. Temp (%.1f C) < Alarme (%.1f C). Transicionando para MONITORANDO.", temperatura_atual, temperatura_alarme);
                    }
                }
                break;

            case ESTADO_ALARME_ATIVO:
                atualizar_leds(temperatura_atual, temperatura_alarme, estado_atual);
                controlar_buzzer(temperatura_atual, temperatura_alarme, estado_atual);
                atualiza_display(temperatura_atual, temperatura_alarme);
                write_file(&temperatura_atual, &temperatura_alarme);

                if (temperatura_atual < temperatura_alarme) {
                    ESP_LOGW(APP_TAG, "Temp (%.1f C) < Alarme (%.1f C). Transicionando para MONITORANDO.", temperatura_atual, temperatura_alarme);
                    estado_atual = ESTADO_MONITORANDO;
                }
                break;

            default:
                ESP_LOGE(APP_TAG, "Estado desconhecido! Recuperando para MONITORANDO.");
                estado_atual = ESTADO_MONITORANDO;
                break;
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main() {
    ESP_LOGI(APP_TAG, "Iniciando a aplicacao de monitoramento de temperatura...");

    configurar_leds();
    configurar_pwm();
    configurar_adc();
    i2c_init();
    lcd_i2c_init(&display);
    configurar_botoes();
    init_sd();

    fila_botoes = xQueueCreate(5, sizeof(tipo_botao_t));
    if (fila_botoes == NULL) {
        ESP_LOGE(APP_TAG, "Falha ao criar a fila de botoes. Abortando...");
        return;
    } else {
        ESP_LOGI(APP_TAG, "Fila de botoes criada com sucesso.");
    }

    xTaskCreate(tarefa_controle_botoes, "Controle_Botoes", 2048, NULL, 10, NULL);
    xTaskCreate(tarefa_principal, "Principal", 8192, NULL, 5, NULL);
}
