#include "esp_types.h"
#include "audio_common.h"
#include "audio_element.h"
#include "audio_error.h"

// typedef struct {
//     bool                     uninstall_drv;      /*!< whether uninstall the i2s driver when stream destroyed*/
// } i2s_stream_cfg_t;

// audio_element_handle_t i2s_stream_init(i2s_stream_cfg_t *config);
// esp_err_t i2s_stream_set_clk(audio_element_handle_t i2s_stream, int rate, int bits, int ch);
audio_element_handle_t analog_stream_init();
