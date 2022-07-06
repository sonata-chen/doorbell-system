#include "audio_common.h"
#include "audio_element.h"
#include "audio_error.h"
#include "audio_mem.h"
#include "driver/timer.h"
#include "esp_err.h"
#include "esp_log.h"

/**/
#include "analogStream.h"
#include "recorder.h"

#define A_STREAM_TASK_STACK (3072 + 512)
#define A_STREAM_BUF_SIZE (2048)
#define A_STREAM_TASK_PRIO (23)
#define A_STREAM_TASK_CORE (0)
#define A_STREAM_RINGBUFFER_SIZE (2 * 1024)

static const char *TAG = "A_STREAM";

static esp_err_t _a_open(audio_element_handle_t self);
static esp_err_t _a_destroy(audio_element_handle_t self);
static esp_err_t _a_close(audio_element_handle_t self);
static int _a_read(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait, void *context);
static int _a_process(audio_element_handle_t self, char *in_buffer, int in_len);

audio_element_handle_t analog_stream_init()
{
    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    audio_element_handle_t el;
    cfg.open = _a_open;
    cfg.close = _a_close;
    cfg.process = _a_process;
    cfg.destroy = _a_destroy;
    cfg.task_stack = A_STREAM_TASK_STACK;
    cfg.task_prio = A_STREAM_TASK_PRIO;
    cfg.task_core = A_STREAM_TASK_CORE;
    cfg.stack_in_ext = false;
    cfg.out_rb_size = A_STREAM_RINGBUFFER_SIZE;
    cfg.multi_out_rb_num = 0;
    cfg.tag = "as";
    cfg.buffer_len = A_STREAM_BUF_SIZE;

    // i2s_stream_t *i2s = audio_calloc(1, sizeof(i2s_stream_t));
    // AUDIO_MEM_CHECK(TAG, i2s, return NULL);
    // memcpy(&i2s->config, config, sizeof(i2s_stream_cfg_t));

    // i2s->type = config->type;
    // i2s->use_alc = config->use_alc;
    // i2s->volume = config->volume;
    // i2s->uninstall_drv = config->uninstall_drv;

    // if (config->type == AUDIO_STREAM_READER) {
    //     cfg.read = _i2s_read;
    // } else if (config->type == AUDIO_STREAM_WRITER) {
    //     cfg.write = _i2s_write;
    // }

    cfg.read = _a_read;

    // if (i2s_driver_install(i2s->config.i2s_port, &i2s->config.i2s_config, 0, NULL) != ESP_OK) {
    //     audio_free(i2s);
    //     return NULL;
    // }

    el = audio_element_init(&cfg);
    AUDIO_MEM_CHECK(TAG, el, {
        return NULL;
    });
    Recorder *recorder = newRecorder(16000, ADC_CHANNEL_5, ADC_WIDTH_BIT_12, TIMER_0);
    audio_element_setdata(el, recorder);

    audio_element_set_music_info(el, 16000, 1, 12);

    return el;
}

static esp_err_t _a_open(audio_element_handle_t self)
{
    return ESP_OK;
}

static esp_err_t _a_close(audio_element_handle_t self)
{
    return ESP_OK;
}

static int _a_read(audio_element_handle_t self, char *buffer, int len, TickType_t ticks_to_wait, void *context)
{
    Recorder *r = audio_element_getdata(self);

    timer_event_t evt;
    int i;
    for (i = 0; i < len; i += 2) {
        BaseType_t ret = xQueueReceive(r->sample_queue, &evt, portMAX_DELAY);

        while (ret != pdTRUE) {
            ret = xQueueReceive(r->sample_queue, &evt, portMAX_DELAY);
        }

        // convert adc value to 16 bits audio sample
        int width = r->width + 9;
        int16_t sample_16_bits = evt.sample * (1 << (16 - width)) - (1 << (16 - 1));
        int8_t higher_byte = sample_16_bits >> 8;
        int8_t lower_byte = sample_16_bits;
        buffer[i + 1] = higher_byte;
        buffer[i + 0] = lower_byte;
    }

    return len;
}

static esp_err_t _a_destroy(audio_element_handle_t self)
{
    Recorder *r = audio_element_getdata(self);

    timer_deinit(TIMER_GROUP_0, r->timer_idx);
    destructRecorder(r);
    return ESP_OK;
}

static int _a_process(audio_element_handle_t self, char *in_buffer, int in_len)
{
    int r_size = audio_element_input(self, in_buffer, in_len);
    int out_len = r_size;
    if (r_size > 0) {
        out_len = audio_element_output(self, in_buffer, r_size);
        if (out_len > 0) {
            audio_element_update_byte_pos(self, out_len);
        }
    }

    return out_len;
}
