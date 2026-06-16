// SmartFactory ESP32 + TCS34725 + 3 Servos
// Versao revisada com melhorias de diagnostico e calibracao

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define I2C_SDA_PIN          21
#define I2C_SCL_PIN          22
#define SERVO_VERMELHO_PIN   25
#define SERVO_AZUL_PIN       26
#define SERVO_VERDE_PIN      27

#define I2C_PORT             I2C_NUM_0
#define I2C_FREQ_HZ          400000

#define TCS_ADDR             0x29
#define TCS_CMD              0x80
#define TCS_CMD_AUTOINC      0x20

#define TCS_REG_ENABLE       0x00
#define TCS_REG_ATIME        0x01
#define TCS_REG_CONTROL      0x0F
#define TCS_REG_CDATAL       0x14

#define TCS_PON              0x01
#define TCS_AEN              0x02
#define TCS_ATIME_154MS      0xC0
#define TCS_GAIN_4X          0x01

#define SERVO_TIMER          LEDC_TIMER_0
#define SERVO_MODE           LEDC_LOW_SPEED_MODE
#define SERVO_RESOLUCAO      LEDC_TIMER_13_BIT
#define SERVO_FREQ_HZ        50

#define CH_VERMELHO          LEDC_CHANNEL_0
#define CH_AZUL              LEDC_CHANNEL_1
#define CH_VERDE             LEDC_CHANNEL_2

#define PORTA_FECHADA_GRAUS  0
#define PORTA_ABERTA_GRAUS   120
#define TEMPO_PORTA_ABERTA   3000

#define LIMIAR_CLEAR_MIN     200
#define LIMIAR_RAZAO         1.4f

static const char *TAG = "SmartFactory";

typedef enum {
    COR_NENHUMA,
    COR_VERMELHA,
    COR_AZUL,
    COR_VERDE
} Cor;

typedef struct {
    uint16_t c, r, g, b;
} RGBC;

static void i2c_escrever_reg(uint8_t reg, uint8_t valor)
{
    uint8_t buf[2] = { TCS_CMD | reg, valor };

    esp_err_t err = i2c_master_write_to_device(
        I2C_PORT, TCS_ADDR, buf, 2, pdMS_TO_TICKS(100));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao escrever reg 0x%02X", reg);
    }
}

static void i2c_ler_reg(uint8_t reg, uint8_t *dados, size_t tamanho)
{
    uint8_t cmd = TCS_CMD | TCS_CMD_AUTOINC | reg;

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT, TCS_ADDR,
        &cmd, 1,
        dados, tamanho,
        pdMS_TO_TICKS(100));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Erro de leitura do sensor");
    }
}

static void i2c_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };

    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

static bool tcs34725_detectado(void)
{
    uint8_t dado;

    esp_err_t err =
        i2c_master_write_read_device(
            I2C_PORT,
            TCS_ADDR,
            &(uint8_t){TCS_CMD | TCS_REG_ENABLE},
            1,
            &dado,
            1,
            pdMS_TO_TICKS(100));

    return err == ESP_OK;
}

static void tcs34725_init(void)
{
    i2c_escrever_reg(TCS_REG_ENABLE, TCS_PON);
    vTaskDelay(pdMS_TO_TICKS(3));

    i2c_escrever_reg(TCS_REG_ENABLE, TCS_PON | TCS_AEN);
    i2c_escrever_reg(TCS_REG_ATIME, TCS_ATIME_154MS);
    i2c_escrever_reg(TCS_REG_CONTROL, TCS_GAIN_4X);

    vTaskDelay(pdMS_TO_TICKS(160));
}

static RGBC tcs34725_ler(void)
{
    uint8_t raw[8];
    i2c_ler_reg(TCS_REG_CDATAL, raw, 8);

    RGBC leitura = {
        .c = (uint16_t)(raw[1] << 8 | raw[0]),
        .r = (uint16_t)(raw[3] << 8 | raw[2]),
        .g = (uint16_t)(raw[5] << 8 | raw[4]),
        .b = (uint16_t)(raw[7] << 8 | raw[6]),
    };

    return leitura;
}

static Cor identificar_cor(RGBC leitura)
{
    if (leitura.c < LIMIAR_CLEAR_MIN)
        return COR_NENHUMA;

    float rn = (float)leitura.r / leitura.c;
    float gn = (float)leitura.g / leitura.c;
    float bn = (float)leitura.b / leitura.c;

    if (rn > gn * LIMIAR_RAZAO && rn > bn * LIMIAR_RAZAO)
        return COR_VERMELHA;

    if (gn > rn * LIMIAR_RAZAO && gn > bn * LIMIAR_RAZAO)
        return COR_VERDE;

    if (bn > rn * LIMIAR_RAZAO && bn > gn * LIMIAR_RAZAO)
        return COR_AZUL;

    return COR_NENHUMA;
}

static uint32_t graus_para_duty(int graus)
{
    uint32_t pulso_us = 500 + (2000 * graus / 180);
    return (pulso_us * 8192) / 20000;
}

static void servo_set_graus(ledc_channel_t canal, int graus)
{
    ledc_set_duty(SERVO_MODE, canal, graus_para_duty(graus));
    ledc_update_duty(SERVO_MODE, canal);
}

static void servos_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = SERVO_MODE,
        .timer_num = SERVO_TIMER,
        .duty_resolution = SERVO_RESOLUCAO,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t canais[3] = {
        {.gpio_num = SERVO_VERMELHO_PIN, .channel = CH_VERMELHO},
        {.gpio_num = SERVO_AZUL_PIN, .channel = CH_AZUL},
        {.gpio_num = SERVO_VERDE_PIN, .channel = CH_VERDE}
    };

    for (int i = 0; i < 3; i++) {
        canais[i].speed_mode = SERVO_MODE;
        canais[i].timer_sel = SERVO_TIMER;
        canais[i].duty = graus_para_duty(PORTA_FECHADA_GRAUS);
        canais[i].hpoint = 0;
        ledc_channel_config(&canais[i]);
    }

    servo_set_graus(CH_VERMELHO, PORTA_FECHADA_GRAUS);
    servo_set_graus(CH_AZUL, PORTA_FECHADA_GRAUS);
    servo_set_graus(CH_VERDE, PORTA_FECHADA_GRAUS);
}

static void abrir_porta(ledc_channel_t canal, const char *nome)
{
    ESP_LOGI(TAG, "Porta %s aberta", nome);

    servo_set_graus(canal, PORTA_ABERTA_GRAUS);
    vTaskDelay(pdMS_TO_TICKS(TEMPO_PORTA_ABERTA));

    servo_set_graus(canal, PORTA_FECHADA_GRAUS);

    ESP_LOGI(TAG, "Porta %s fechada", nome);
}

void app_main(void)
{
    i2c_init();

    if (!tcs34725_detectado()) {
        ESP_LOGE(TAG, "TCS34725 nao encontrado");

        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    tcs34725_init();
    servos_init();

    Cor cor_anterior = COR_NENHUMA;

    while (1) {

        RGBC leitura = tcs34725_ler();
        Cor cor = identificar_cor(leitura);

        ESP_LOGI(TAG,
                 "C=%u R=%u G=%u B=%u",
                 leitura.c,
                 leitura.r,
                 leitura.g,
                 leitura.b);

        if (cor != COR_NENHUMA &&
            cor != cor_anterior) {

            switch (cor) {

                case COR_VERMELHA:
                    abrir_porta(CH_VERMELHO, "VERMELHA");
                    break;

                case COR_AZUL:
                    abrir_porta(CH_AZUL, "AZUL");
                    break;

                case COR_VERDE:
                    abrir_porta(CH_VERDE, "VERDE");
                    break;

                default:
                    break;
            }
        }

        cor_anterior = cor;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
