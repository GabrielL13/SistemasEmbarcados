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

#define RS 0
#define EN 2
#define BL 3

#define CLEAR_DISPLAY 0x01
#define SET_4BIT_MODE 0x28
#define DISPLAY_ON_CURSOR_OFF 0x0C
#define CURSOR_RIGHT_NO_SHIFT_LEFT 0x06

#define NTC_SERIES_RESISTOR 10000
#define NTC_BETA 3950
#define NTC_NOMINAL_RESISTANCE 10000
#define NTC_NOMINAL_TEMPERATURE 25.0f
#define NUM_AMOSTRAS_NTC 10
#define INTERVALO_LEITURA_MS 1000 

// --- Definições para SD Card SPI ---
#define PIN_NUM_MISO GPIO_NUM_16
#define PIN_NUM_MOSI GPIO_NUM_17
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_CS GPIO_NUM_15

#define MOUNT_POINT "/sdcard"

static const char *APP_TAG = "MONITOR_TEMP_SD";
volatile uint32_t ultima_vez_inc = 0;
volatile uint32_t ultima_vez_dec = 0;
const TickType_t debounce_ticks = pdMS_TO_TICKS(200);
volatile float temperatura_alarme = 25.0f;
QueueHandle_t fila_botoes;

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
  .address = 0x27,
  .num = I2C_NUM_1,
  .backlight = 1,
  .size = 0
};

sdmmc_card_t *card;
static sdmmc_host_t host = SDSPI_HOST_DEFAULT();

void init_sd() {
  esp_err_t ret;
  spi_bus_config_t bus_cfg = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = PIN_NUM_MISO,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1, 
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
  };
  ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK) {
    ESP_LOGE(APP_TAG, "Falha ao inicializar o barramento SPI para SD (%s).", esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(APP_TAG, "Barramento SPI para SD inicializado.");

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = PIN_NUM_CS;
  slot_config.host_id = host.slot;

  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    .format_if_mount_failed = false, 
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
  };

  ESP_LOGI(APP_TAG, "Montando o sistema de arquivos no %s", MOUNT_POINT);
  ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      ESP_LOGE(APP_TAG, "Falha ao montar o sistema de arquivos. O SD card pode estar corrompido ou não formatado. Você pode tentar formatar a placa.");
    } else {
      ESP_LOGE(APP_TAG, "Falha ao inicializar a placa SD (%s). Verifique as conexões e o pinout.", esp_err_to_name(ret));
    }
    return;
  }
  ESP_LOGI(APP_TAG, "Sistema de arquivos montado com sucesso.");
  sdmmc_card_print_info(stdout, card);
}

void close_sd() {
  esp_vfs_fat_sdmmc_unmount();
  ESP_LOGI(APP_TAG, "SD Card desmontado.");
  spi_bus_free(host.slot);
  ESP_LOGI(APP_TAG, "Barramento SPI liberado.");
}

void write_file(float *temp_atual, float *temp_alarme) {
  ESP_LOGI(APP_TAG, "Abrindo arquivo %s para escrita...", MOUNT_POINT"/log.txt");
  FILE *f = fopen(MOUNT_POINT"/log.txt", "a");
  if (f == NULL) {
    ESP_LOGE(APP_TAG, "Falha ao abrir arquivo para escrita");
    return;
  }
  fprintf(f, "Temperatura Atual: %.1f C, Alarme: %.1f C\n", *temp_atual, *temp_alarme);
  fclose(f);
  ESP_LOGI(APP_TAG, "Dados escritos no arquivo log.txt.");
}

void read_file() {
  ESP_LOGI(APP_TAG, "Lendo arquivo %s...", MOUNT_POINT"/log.txt");
  FILE *f = fopen(MOUNT_POINT"/log.txt", "r");
  if (f == NULL) {
    ESP_LOGE(APP_TAG, "Falha ao abrir arquivo para leitura");
    return;
  }
  char line[128];
  while (fgets(line, sizeof(line), f) != NULL) {
    ESP_LOGI(APP_TAG, "Lido do SD: %s", line);
  }
  fclose(f);
  ESP_LOGI(APP_TAG, "Leitura do arquivo concluída.");
}

void configurar_leds() {
  gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
  gpio_set_direction(LED4, GPIO_MODE_OUTPUT);
  ESP_LOGI(APP_TAG, "LEDs configurados.");
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
  ESP_LOGI(APP_TAG, "PWM para buzzer configurado.");
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
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = SCL_PIN,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
  };
  i2c_param_config(display.num, &conf);
  i2c_driver_install(display.num, conf.mode, 0, 0, 0);
  ESP_LOGI(APP_TAG, "I2C inicializado.");
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
  ESP_LOGI(APP_TAG, "LCD I2C inicializado.");
}

void lcd_i2c_cursor_set(lcd_i2c_handle_t *lcd, uint8_t col, uint8_t row) {
  const uint8_t row_offsets[] = {0x00, 0x40};
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

float ler_temp_filtrada() {
  long sum_leitura = 0;
  for (int i = 0; i < NUM_AMOSTRAS_NTC; i++) {
    sum_leitura += adc1_get_raw(NTC_ADC_CHANNEL);
    esp_rom_delay_us(100);
  }
  int leitura = sum_leitura / NUM_AMOSTRAS_NTC;

  ESP_LOGD(APP_TAG, "ADC RAW (filtrado): %d", leitura);

  if (leitura <= 0 || leitura >= 4095) { 
    ESP_LOGE(APP_TAG, "Leitura ADC invalida ou fora da faixa.");
    return NAN; 
  }
  float resistencia = (float)NTC_SERIES_RESISTOR * leitura / (4095.0f - leitura);
  float tempK = 1.0f / (1.0f / (NTC_NOMINAL_TEMPERATURE + 273.15f) + log(resistencia / NTC_NOMINAL_RESISTANCE) / NTC_BETA);
  float tempC = tempK - 273.15f;
  return tempC;
}

void atualiza_display(float temp, float alarme) {
  lcd_i2c_cursor_set(&display, 0, 0);
  lcd_i2c_print(&display, "                "); 
  lcd_i2c_cursor_set(&display, 0, 0);
  if (isnan(temp)) {
    lcd_i2c_print(&display, "Temp: ---.- C");
  } else {
    lcd_i2c_print(&display, "Temp: %.1f C", temp);
  }
  lcd_i2c_cursor_set(&display, 0, 1);
  lcd_i2c_print(&display, "                "); 
  lcd_i2c_cursor_set(&display, 0, 1);
  lcd_i2c_print(&display, "Alarme: %.1f C", alarme);
}

void atualizar_leds(float temp, float alarme) {
  float diferenca = fabs(alarme - temp);

  if (temp >= alarme) {
    static bool piscar_estado = false;
    piscar_estado = !piscar_estado; 
    gpio_set_level(LED1, piscar_estado);
    gpio_set_level(LED2, piscar_estado);
    gpio_set_level(LED3, piscar_estado);
    gpio_set_level(LED4, piscar_estado);
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

void controlar_buzzer(float temp, float alarme) {
  if (temp >= alarme) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  } else {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0); 
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  }
}

void IRAM_ATTR trata_botao_inc() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t agora = xTaskGetTickCountFromISR();

  if (agora - ultima_vez_inc > debounce_ticks) {
    tipo_botao_t botao = INCREMENTA;
    xQueueSendFromISR(fila_botoes, &botao, &xHigherPriorityTaskWoken);
    ultima_vez_inc = agora;
  }

  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void IRAM_ATTR trata_botao_dec() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t agora = xTaskGetTickCountFromISR();

  if (agora - ultima_vez_dec > debounce_ticks) {
    tipo_botao_t botao = DECREMENTA;
    xQueueSendFromISR(fila_botoes, &botao, &xHigherPriorityTaskWoken);
    ultima_vez_dec = agora;
  }

  if (xHigherPriorityTaskWoken) {
    portYIELD_FROM_ISR();
  }
}

void configurar_botoes() {
  gpio_set_direction(BOTAO_INC, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BOTAO_INC, GPIO_PULLUP_ONLY);
  gpio_set_intr_type(BOTAO_INC, GPIO_INTR_NEGEDGE);
  gpio_install_isr_service(0); 
  gpio_isr_handler_add(BOTAO_INC, trata_botao_inc, NULL);

  gpio_set_direction(BOTAO_DEC, GPIO_MODE_INPUT);
  gpio_set_pull_mode(BOTAO_DEC, GPIO_PULLUP_ONLY);
  gpio_set_intr_type(BOTAO_DEC, GPIO_INTR_NEGEDGE);
  gpio_isr_handler_add(BOTAO_DEC, trata_botao_dec, NULL);
  ESP_LOGI(APP_TAG, "Botões configurados.");
}

void tarefa_controle_botoes(void *pvParameters) {
  tipo_botao_t botao;
  while (1) {
    if (xQueueReceive(fila_botoes, &botao, portMAX_DELAY)) {
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
    atualizar_leds(temperatura_atual, temperatura_alarme);
    controlar_buzzer(temperatura_atual, temperatura_alarme);
    atualiza_display(temperatura_atual, temperatura_alarme);
    write_file(&temperatura_atual, &temperatura_alarme);
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
  init_sd(); // Inicializa o SD Card

  fila_botoes = xQueueCreate(5, sizeof(tipo_botao_t));
  if (fila_botoes == NULL) {
    ESP_LOGE(APP_TAG, "Falha ao criar a fila de botoes. Abortando...");
    return;
  } else {
    ESP_LOGI(APP_TAG, "Fila de botoes criada com sucesso.");
  }
  xTaskCreate(tarefa_controle_botoes, "Controle_Botoes", 2048, NULL, 10, NULL);
  xTaskCreate(tarefa_principal, "Principal", 8192, NULL, 5, NULL);

  ESP_LOGI(APP_TAG, "Aplicacao iniciada. Monitorando temperatura...");
}
