/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_board_init.h"
#include "driver/i2s.h"
#include "model_path.h"
#include "ringbuf.h"

#include "esp_system.h"
#include "esp_event.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp32s3/rom/gpio.h"
#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "esp_spi_flash.h"


#define DEBUG_SAVE_PCM      0

#define GPIO_HANDSHAKE 18
#define GPIO_MOSI 8
#define GPIO_MISO 20
#define GPIO_SCLK 19
#define GPIO_CS 6
#define RCV_HOST SPI2_HOST
#define DMA_CHAN SPI_DMA_CH_AUTO

#define FRAME_SIZE  640


#if DEBUG_SAVE_PCM
#define FILES_MAX           3
ringbuf_handle_t rb_debug[FILES_MAX] = {NULL};
FILE * file_save[FILES_MAX] = {NULL};
#endif

volatile static int start_flag = 0;
int detect_flag = 0;
static esp_afe_sr_iface_t *afe_handle = NULL;

static ringbuf_handle_t communicate_rb = NULL;


// SPI start
//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TS_REG, (1 << GPIO_HANDSHAKE));
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans) {
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, (1 << GPIO_HANDSHAKE));
}

static void communicate_spi(void * ard)
{
    int n = 0;
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096 * 8,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    //Configuration for the handshake line
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1 << GPIO_HANDSHAKE)
    };

    //Configure handshake line as output
    gpio_config(&io_conf);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);

    //Initialize SPI slave interface
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, DMA_CHAN);
    printf("%d\n", ret);
    assert(ret == ESP_OK);

    WORD_ALIGNED_ATTR char sendbuf_s[6] = "start!";
    WORD_ALIGNED_ATTR char *sendbuf = heap_caps_malloc(FRAME_SIZE * sizeof(int16_t), MALLOC_CAP_DMA);

    spi_slave_transaction_t t;
    memset(&t, 0, sizeof(t));
    size_t i2s_bytes_write = 0;

    while (1) {
        if (start_flag == 0 || start_flag == -1) {
            t.tx_buffer = sendbuf_s;
            t.rx_buffer = NULL;
            ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
            printf("\n\n---------@@@-------\n\n");
            vTaskDelay(100 / portTICK_PERIOD_MS);

            rb_reset(communicate_rb);
            start_flag = 1;
        } else {
            rb_read(communicate_rb, sendbuf, FRAME_SIZE * sizeof(int16_t), portMAX_DELAY);

            t.length = FRAME_SIZE * sizeof(int16_t) * 8;
            t.tx_buffer = sendbuf;
            t.rx_buffer = NULL;

            ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
        }
    }
}
// SPI end

void feed_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int nch = afe_handle->get_total_channel_num(afe_data);
    int feed_channel = esp_get_feed_channel();
    int16_t *i2s_buff = malloc(audio_chunksize * sizeof(int16_t) * feed_channel);
    assert(i2s_buff);
    size_t bytes_read;

    while (1) {
        // esp_get_feed_data(i2s_buff, audio_chunksize * sizeof(int16_t) * feed_channel);
        i2s_read(I2S_NUM_1, i2s_buff, audio_chunksize * feed_channel * sizeof(int16_t), &bytes_read, portMAX_DELAY);

        for (int i = 0; i < audio_chunksize; i++) {
            int16_t ref = i2s_buff[4 * i + 2];
            i2s_buff[3 * i + 0] = i2s_buff[4 * i + 1];
            i2s_buff[3 * i + 1] = i2s_buff[4 * i + 3];
            i2s_buff[3 * i + 2] = ref;
            // i2s_buff[2 * i + 0] = i2s_buff[4 * i + 1];
            // i2s_buff[2 * i + 1] = ref;
        }

        afe_handle->feed(afe_data, i2s_buff);

    #if DEBUG_SAVE_PCM
        if (rb_bytes_available(rb_debug[0]) < audio_chunksize * nch * sizeof(int16_t)) {
            printf("ERROR! rb_debug[0] slow!!!\n");
        }

        rb_write(rb_debug[0], i2s_buff, audio_chunksize * nch * sizeof(int16_t), 0);
    #endif
    }
    afe_handle->destroy(afe_data);
    vTaskDelete(NULL);
}

void detect_Task(void *arg)
{
    esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    int16_t *buff = malloc(afe_chunksize * sizeof(int16_t));
    assert(buff);
    printf("------------detect start------------\n");

    while (1) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data); 
        if (!res || res->ret_value == ESP_FAIL) {
            printf("fetch error!\n");
            break;
        }

    #if DEBUG_SAVE_PCM
        if (rb_bytes_available(rb_debug[1]) < afe_chunksize * 1 * sizeof(int16_t)) {
            printf("ERROR! rb_debug[1] slow!!!\n");
        }

        rb_write(rb_debug[1], res->data, afe_chunksize * 1 * sizeof(int16_t), 0);
    #endif

        if (rb_bytes_available(communicate_rb) < afe_chunksize * 1 * sizeof(int16_t)) {
            // printf("ERROR! communicate_rb slow!!!\n");
        }
        rb_write(communicate_rb, res->data, afe_chunksize * 1 * sizeof(int16_t), 0);

        if (res->wakeup_state == WAKENET_DETECTED) {
            printf("wakeword detected\n");
            printf("-----------LISTENING-----------\n");
        }
    }
    afe_handle->destroy(afe_data);
    if (buff) {
        free(buff);
        buff = NULL;
    }
    vTaskDelete(NULL);
}

#if DEBUG_SAVE_PCM
void debug_pcm_save_Task(void *arg)
{
    int size = 4096;   // 4k bytes
    int16_t *buf_temp = heap_caps_calloc(1, size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    while (1) {
        for (int i = 0; i < FILES_MAX; i++) {
            if (file_save[i] != NULL) {
                if (rb_bytes_filled(rb_debug[i]) > size) {
                    int ret = rb_read(rb_debug[i], buf_temp, size, 3000 / portTICK_PERIOD_MS);
                    if ((ret < 0) || (ret < size)) {
                        // ESP_LOGE(TAG, "rb_debug read error, ret: %d\n", ret);
                        vTaskDelay(10 / portTICK_RATE_MS);
                        continue;
                    }
                    FatfsComboWrite(buf_temp, size, 1, file_save[i]);
                }
            }
        }
        vTaskDelay(1 / portTICK_RATE_MS);
    }

    free(buf_temp);
    vTaskDelete(NULL);
}
#endif

void app_main()
{
    ESP_ERROR_CHECK(esp_board_init(AUDIO_HAL_08K_SAMPLES, 1, 16));
#if DEBUG_SAVE_PCM
    ESP_ERROR_CHECK(esp_sdcard_init("/sdcard", 10));
#endif

    srmodel_list_t *models = esp_srmodel_init("model");
    if (models!=NULL) {
        for (int i=0; i<models->num; i++) {
            printf("Load: %s\n", models->model_name[i]);
        }
    }
    char *wn_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);

    afe_handle = &ESP_AFE_SR_HANDLE;
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    afe_config.memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_config.vad_init = false;
    afe_config.wakenet_init = true;
    afe_config.wakenet_model_name = wn_name;
    afe_config.voice_communication_init = false;
    afe_config.agc_mode = AFE_MN_PEAK_NO_AGC;

    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);

#if DEBUG_SAVE_PCM
    rb_debug[0] = rb_create(afe_handle->get_total_channel_num(afe_data) * 4 * 16000 * 2, 1);   // 4s ringbuf
    file_save[0] = fopen("/sdcard/feed.pcm", "w");
    if (file_save[0] == NULL) printf("can not open file\n");

    rb_debug[1] = rb_create(1 * 4 * 16000 * 2, 1);   // 4s ringbuf
    file_save[1] = fopen("/sdcard/fetch.pcm", "w");
    if (file_save[1] == NULL) printf("can not open file\n");

    xTaskCreatePinnedToCore(&debug_pcm_save_Task, "debug_pcm_save", 2 * 1024, NULL, 5, NULL, 1);
#endif

    communicate_rb = rb_create(1 * 2 * 16000 * 2, 1);   // 2s ringbuf

    xTaskCreatePinnedToCore(&feed_Task, "feed", 8 * 1024, (void*)afe_data, 5, NULL, 0);
    xTaskCreatePinnedToCore(&detect_Task, "detect", 4 * 1024, (void*)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&communicate_spi, "communicate_spi", 3 * 1024, NULL, 5, NULL, 0);
}
