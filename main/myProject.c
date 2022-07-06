#include "freertos/FreeRTOS.h"

/**/
#include "algorithm_stream.h"
#include "audio_common.h"
#include "audio_element.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_pipeline.h"
#include "board.h"
#include "esp_log.h"
#include "esp_peripherals.h"
#include "esp_sip.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "filter_resample.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "g711_decoder.h"
#include "g711_encoder.h"
#include "i2s_stream.h"
#include "input_key_service.h"
#include "nvs_flash.h"
#include "periph_wifi.h"
#include "raw_stream.h"

/**/
#include "analogStream.h"
#include "recorder.h"

#if __has_include("esp_idf_version.h")
#include "esp_idf_version.h"
#else
#define ESP_IDF_VERSION_VAL(major, minor, patch) 1
#endif

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0))
#include "esp_netif.h"
#else
#include "tcpip_adapter.h"
#endif

static const char *TAG = "VOIP_EXAMPLE";  // Logging message

static sip_handle_t sip;  // SIP 服務
static audio_element_handle_t raw_read, raw_write; // Raw Stream
static audio_pipeline_handle_t recorder, player;   // Audio Pipeline

/* 
 *  Button Event handle
 *
 *  當實體按鈕被按下時，會觸發 gpio 中斷，被觸發的腳位的號碼會被加入
 *  queue 中，因此有多個按鈕時可以知道是哪個按鈕被按下，但目前我們只有
 *  一個按鈕。
 *
 *  gpio_task() 負責處理 gpio 中斷，它會呼叫 call() 函式來撥打網路電話。
 */

static xQueueHandle gpio_evt_queue = NULL; // queue

static void call()
{
    ESP_LOGI(TAG, "handler");
    esp_sip_uac_invite(sip, "200");
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;  // 腳位號碼
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task(void *arg)
{
    uint32_t io_num;  // 腳位號碼
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            call();
        }
    }
}

/* Create audio pipeline for recorder */
static esp_err_t recorder_pipeline_open()
{
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    recorder = audio_pipeline_init(&pipeline_cfg);
    AUDIO_NULL_CHECK(TAG, recorder, return ESP_FAIL);

    audio_element_handle_t a_stream = analog_stream_init();

    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 16000;
    rsp_cfg.src_ch = 1;
    rsp_cfg.dest_rate = 8000;
    rsp_cfg.dest_ch = 1;
    rsp_cfg.complexity = 5;
    rsp_cfg.task_core = 1;
    audio_element_handle_t filter = rsp_filter_init(&rsp_cfg);

    g711_encoder_cfg_t g711_cfg = DEFAULT_G711_ENCODER_CONFIG();
    g711_cfg.task_core = 1;
    g711_cfg.enc_mode = 1; /* 0: a-law  1: u-law */
    audio_element_handle_t sip_encoder = g711_encoder_init(&g711_cfg);

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_READER;
    raw_read = raw_stream_init(&raw_cfg);
    audio_element_set_output_timeout(raw_read, portMAX_DELAY);

    audio_pipeline_register(recorder, a_stream, "as");
    audio_pipeline_register(recorder, filter, "filter");
    audio_pipeline_register(recorder, sip_encoder, "sip_enc");
    audio_pipeline_register(recorder, raw_read, "raw");

    const char *link_tag[4] = {"as", "filter", "sip_enc", "raw"};
    audio_pipeline_link(recorder, &link_tag[0], 4);

    audio_pipeline_run(recorder);
    ESP_LOGI(TAG, " SIP recorder has been created");
    return ESP_OK;
}

/* Create audio pipeline for player */
static esp_err_t player_pipeline_open()
{
    audio_element_handle_t i2s_stream_writer;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    player = audio_pipeline_init(&pipeline_cfg);
    AUDIO_NULL_CHECK(TAG, player, return ESP_FAIL);

    raw_stream_cfg_t raw_cfg = RAW_STREAM_CFG_DEFAULT();
    raw_cfg.type = AUDIO_STREAM_WRITER;
    raw_write = raw_stream_init(&raw_cfg);

    g711_decoder_cfg_t g711_cfg = DEFAULT_G711_DECODER_CONFIG();
    g711_cfg.dec_mode = 1; /* 0: a-law  1: u-law */
    audio_element_handle_t sip_decoder = g711_decoder_init(&g711_cfg);

    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 8000;
    rsp_cfg.src_ch = 1;
    rsp_cfg.dest_rate = 8000;
    rsp_cfg.dest_ch = 1;
    rsp_cfg.complexity = 5;
    audio_element_handle_t filter = rsp_filter_init(&rsp_cfg);

    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_INTERNAL_DAC_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_cfg.uninstall_drv = false;
    i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    i2s_cfg.i2s_config.sample_rate = 8000;
    i2s_cfg.i2s_config.communication_format = I2S_COMM_FORMAT_I2S;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    audio_pipeline_register(player, raw_write, "raw");
    audio_pipeline_register(player, sip_decoder, "sip_dec");
    audio_pipeline_register(player, filter, "filter");
    audio_pipeline_register(player, i2s_stream_writer, "i2s");
    const char *link_tag[4] = {"raw", "sip_dec", "filter", "i2s"};
    audio_pipeline_link(player, &link_tag[0], 4);
    audio_pipeline_run(player);
    ESP_LOGI(TAG, "SIP player has been created");
    return ESP_OK;
}

static ip4_addr_t _get_network_ip()
{
    tcpip_adapter_ip_info_t ip;
    tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip);
    return ip.ip;
}

static int _sip_event_handler(sip_event_msg_t *event)
{
    ip4_addr_t ip;
    switch ((int)event->type) {
        case SIP_EVENT_REQUEST_NETWORK_STATUS:
            ESP_LOGD(TAG, "SIP_EVENT_REQUEST_NETWORK_STATUS");
            ip = _get_network_ip();
            if (ip.addr) {
                return true;
            }
            return ESP_OK;
        case SIP_EVENT_REQUEST_NETWORK_IP:
            ESP_LOGD(TAG, "SIP_EVENT_REQUEST_NETWORK_IP");
            ip = _get_network_ip();
            int ip_len = sprintf((char *)event->data, "%s", ip4addr_ntoa(&ip));
            return ip_len;
        case SIP_EVENT_REGISTERED:
            ESP_LOGI(TAG, "SIP_EVENT_REGISTERED");
            break;
        case SIP_EVENT_RINGING:
            ESP_LOGI(TAG, "ringing... RemotePhoneNum %s", (char *)event->data);
            break;
        case SIP_EVENT_INVITING:
            ESP_LOGI(TAG, "SIP_EVENT_INVITING Remote Ring...");
            break;
        case SIP_EVENT_BUSY:
            ESP_LOGI(TAG, "SIP_EVENT_BUSY");
            break;
        case SIP_EVENT_HANGUP:
            ESP_LOGI(TAG, "SIP_EVENT_HANGUP");
            break;
        case SIP_EVENT_AUDIO_SESSION_BEGIN:
            ESP_LOGI(TAG, "SIP_EVENT_AUDIO_SESSION_BEGIN");
            player_pipeline_open();
            recorder_pipeline_open();
            break;
        case SIP_EVENT_AUDIO_SESSION_END:
            ESP_LOGI(TAG, "SIP_EVENT_AUDIO_SESSION_END");
            audio_pipeline_stop(player);
            audio_pipeline_wait_for_stop(player);
            audio_pipeline_deinit(player);
            audio_pipeline_stop(recorder);
            audio_pipeline_wait_for_stop(recorder);
            audio_pipeline_deinit(recorder);
            break;
        case SIP_EVENT_READ_AUDIO_DATA:
            return raw_stream_read(raw_read, (char *)event->data, event->data_len);
        case SIP_EVENT_WRITE_AUDIO_DATA:
            return raw_stream_write(raw_write, (char *)event->data, event->data_len);
        case SIP_EVENT_READ_DTMF:
            ESP_LOGI(TAG, "SIP_EVENT_READ_DTMF ID : %d ", ((char *)event->data)[0]);
            break;
    }
    return 0;
}

void app_main()
{
    const char *WIFI_SSID = "test";
    const char *WIFI_PASSWORD = "00000000";
    const char *SIP_URI = "udp://100:100@192.168.7.1:5060";

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("SIP", ESP_LOG_INFO);

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 1, 0))
    ESP_ERROR_CHECK(esp_netif_init());
#else
    tcpip_adapter_init();
#endif

    ESP_LOGI(TAG, "[1.0] Initialize peripherals management");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);

    ESP_LOGI(TAG, "[1.1] Initialize and start peripherals");

    /*  on board led */
    gpio_set_direction(2, GPIO_MODE_OUTPUT);

    /* 設置 adc1 channel 5，用於讀取麥克風的輸出 */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_6);

    ESP_LOGI(TAG, "[1.2] Start and wait for Wi-Fi network");
    periph_wifi_cfg_t wifi_cfg = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASSWORD,
    };
    esp_periph_handle_t wifi_handle = periph_wifi_init(&wifi_cfg);
    esp_periph_start(set, wifi_handle);
    periph_wifi_wait_for_connected(wifi_handle, portMAX_DELAY);


    ESP_LOGI(TAG, "[ 2 ] Create SIP Service");
    sip_config_t sip_cfg = {
        .uri = SIP_URI,
        .event_handler = _sip_event_handler,
        .send_options = true,
        .acodec_type = SIP_ACODEC_G711U, // or SIP_ACODEC_G711A
    };

    /* 啟動 SIP 服務 */
    sip = esp_sip_init(&sip_cfg);
    esp_sip_start(sip);

    ESP_LOGI(TAG, "[ 3 ] Add ISR handler for GPIO pin");
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_INPUT);
    gpio_intr_enable(GPIO_NUM_12); // 啟用 gpio 中斷
    gpio_set_intr_type(GPIO_NUM_12, GPIO_INTR_POSEDGE); // 正緣觸發

    /* 註冊 gpio interupt handler */
    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_12, gpio_isr_handler, (void *)GPIO_NUM_12);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t)); // Create a queue
    xTaskCreate(gpio_task, "gpio_task_example", 2048, NULL, 10, NULL); // Start a task
}
