#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_log.h"
#include "../components/wm8978/include/wm8978.h" //FIXME

#define SAMPLE_RATE (48000)
#define I2S_NUM (0)
#define I2S_BCK_IO (GPIO_NUM_33)
#define I2S_WS_IO (GPIO_NUM_25)
#define I2S_DO_IO (GPIO_NUM_26)
#define I2S_DI_IO (GPIO_NUM_27)

const short int SIN64[64] = {0, 3136, 6242, 9289, 12245, 15084, 17778, 20300, 22627, 24736, 26607, 28221, 29564, 30622, 31385, 31845, 32000,
                             31845, 31385, 30622, 29564, 28221, 26607, 24736, 22627, 20300, 17778, 15084, 12245, 9289, 6242, 3136, 0,
                             -3137, -6243, -9290, -12246, -15085, -17779, -20301, -22628, -24737, -26608, -28222, -29565, -30623, -31386, -31846, -32000,
                             -31846, -31386, -30623, -29565, -28222, -26608, -24737, -22628, -20301, -17779, -15085, -12246, -9290, -6243, -3137};

/* Task plays 100ms long beeps on left channel every second. */
void audio_play_task(void *pvParameter)
{
    signed short samples[256];
    size_t written;
    int total_chunks = 375;

    // fs = 48000 kHz
    // SIN64 - frequency: fs / 64 = 750 Hz
    // We need to generate N samples per second: fs*2 = 96000
    // each time we send 256 so total amount of chuncks per second is 96000/256=375.
    // samples (i&1)==0 (0, 2, 4, ...) go to the left channel
    // samples (i&1)==1 (1, 3, 5, ...) go to the right channel
    while (1)
    {
        for (int j = 0; j < total_chunks; j++)
        {
            for (int i = 0; i < 256; i++)
            {
                if ((i & 1) == 0)
                {
                    samples[i] = SIN64[((i >> 1) & 63)] * (j < 37);
                }
                else
                {
                    samples[i] = 0;
                }
            }
            i2s_write(I2S_NUM, (const char *)samples, 512, &written, portMAX_DELAY);
        }
    }
}

static i2s_config_t i2s_config = {
    .mode = I2S_MODE_MASTER | I2S_MODE_TX,
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB,
    .dma_buf_count = 3,
    .dma_buf_len = 512,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1};

static i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_IO,
    .ws_io_num = I2S_WS_IO,
    .data_out_num = I2S_DO_IO,
    .data_in_num = I2S_DI_IO};

void app_main(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << GPIO_NUM_0;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_0, 0);

    WM8978_Init();
    WM8978_ADDA_Cfg(1, 1);
    WM8978_Input_Cfg(1, 0, 0);
    WM8978_Output_Cfg(1, 0);
    WM8978_MIC_Gain(0);
    WM8978_AUX_Gain(0);
    WM8978_LINEIN_Gain(0);
    WM8978_SPKvol_Set(63);
    WM8978_HPvol_Set(63, 63); //0-63
    WM8978_EQ_3D_Dir(0);
    WM8978_EQ1_Set(0, 24);
    WM8978_EQ2_Set(0, 24);
    WM8978_EQ3_Set(0, 24);
    WM8978_EQ4_Set(0, 24);
    WM8978_EQ5_Set(0, 24);
    WM8978_I2S_Cfg(2, 0);

    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
    REG_WRITE(PIN_CTRL, 0xFFFFFFF0);

    xTaskCreate(&audio_play_task, "audio_play_task", 4096, NULL, 8, NULL);

    while (1)
    {
        vTaskDelay(5000 / portTICK_RATE_MS);
        ESP_LOGI("app", "Heart beat");
    }
}
