#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void telnet_com_start(void);
esp_err_t telnet_send_date(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif
