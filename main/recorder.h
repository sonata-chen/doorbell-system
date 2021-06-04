#include "driver/adc.h"
#include "driver/timer.h"
#include "driver/periph_ctrl.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


#define TIMER_DIVIDER 2                              //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_INTERVAL0_SEC (3.4179)                  // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC (5.78)                    // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD 0                         // testing will be done without auto reload
#define TEST_WITH_RELOAD 1                            // testing will be done with auto reload
/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    // int type;  // the type of timer's event
    // int timer_group;
    // int timer_idx;
    // uint64_t timer_counter_value;
    int16_t sample;
} timer_event_t;

typedef struct recorder {
    adc_channel_t channel;
    adc_bits_width_t width;
    int timer_idx;
    uint64_t samplerate;
    xQueueHandle sample_queue;
} Recorder;

// xQueueHandle timer_queue;

Recorder *newRecorder(uint32_t samplerate, adc_channel_t channel, adc_bits_width_t width, int timer_idx);
void destructRecorder(Recorder *t);

/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\n", (uint32_t)(counter_value >> 32),
           (uint32_t)(counter_value));
    printf("Time   : %.8f s\n", (double)counter_value / TIMER_SCALE);
}
