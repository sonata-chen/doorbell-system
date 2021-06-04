#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "audio_mem.h"

/**/
#include "recorder.h"

static void timer_group0_isr(void *para);
static void recorder_timer_init(Recorder *r, bool auto_reload, double timer_interval_sec);

Recorder *newRecorder(uint32_t samplerate, adc_channel_t channel, adc_bits_width_t width, int timer_idx)
{
    Recorder *r = audio_calloc(1, sizeof(Recorder));

    r->samplerate = samplerate;
    r->channel = channel;
    r->width = width;
    r->timer_idx = timer_idx;

    r->sample_queue = xQueueCreate(2048, sizeof(timer_event_t));

    recorder_timer_init(r, true, 1.0 / samplerate);

    return r;
}

void destructRecorder(Recorder *r)
{
    vQueueDelete(r->sample_queue);
    audio_free(r);
}

/*
 * Timer group0 ISR handler
 *
 * Note:
 * We don't call the timer API here because they are not declared with IRAM_ATTR.
 * If we're okay with the timer irq not being serviced while SPI flash cache is disabled,
 * we can allocate this interrupt without the ESP_INTR_FLAG_IRAM flag and use the normal API.
 */
static void IRAM_ATTR timer_group0_isr(void *para)
{
    timer_spinlock_take(TIMER_GROUP_0);
    Recorder *r = (Recorder *)para;

    /* Retrieve the interrupt status and the counter value
       from the timer that reported the interrupt */
    // uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_0);
    // uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, r->timer_idx);

    /* Prepare basic event data
       that will be then sent back to the main program task */
    timer_event_t evt;
    // evt.timer_group = 0;
    // evt.timer_idx = r->timer_idx;
    // evt.timer_counter_value = timer_counter_value;

    // read digital value from microphone
    int32_t adc_reading = 0;
    adc_reading = adc1_get_raw((adc1_channel_t)(r->channel));
    evt.sample = adc_reading;

    /* Clear the interrupt
       and update the alarm time for the timer with without reload */

    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, r->timer_idx);

    // if (timer_intr & TIMER_INTR_T1) {
    //     evt.type = TEST_WITHOUT_RELOAD;
    //     timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    //     timer_counter_value += (uint64_t)(TIMER_INTERVAL0_SEC * TIMER_SCALE);
    //     timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, r->timer_idx, timer_counter_value);
    // } else if (timer_intr & TIMER_INTR_T0) {
    //     evt.type = TEST_WITH_RELOAD;
    //     timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    // } else {
    //     evt.type = -1;  // not supported even type
    // }

    /* After the alarm has been triggered
      we need enable it again, so it is triggered the next time */
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, r->timer_idx);

    /* Now just send the event data back to the main program task */
    BaseType_t ret = xQueueSendFromISR(r->sample_queue, &evt, NULL);

    if (ret != pdTRUE)
        // ESP_LOGI("RECORDER", "lost");
        gpio_set_level(2, 1);
    else
        gpio_set_level(2, 0);

    timer_spinlock_give(TIMER_GROUP_0);
}

/*
 * Initialize selected timer of the timer group 0
 *
 * r - recorder
 * auto_reload - should the timer auto reload on alarm?
 * timer_interval_sec - the interval of alarm to set
 */
static void recorder_timer_init(Recorder *r, bool auto_reload, double timer_interval_sec)
{
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = auto_reload,
    };  // default clock source is APB
    timer_init(TIMER_GROUP_0, r->timer_idx, &config);

    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, r->timer_idx, 0x00000000ULL);

    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(TIMER_GROUP_0, r->timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, r->timer_idx);
    timer_isr_register(TIMER_GROUP_0, r->timer_idx, timer_group0_isr,
                       (void *)r, ESP_INTR_FLAG_IRAM, NULL);

    timer_start(TIMER_GROUP_0, r->timer_idx);
}
