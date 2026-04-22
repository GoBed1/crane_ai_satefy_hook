#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include "board.h"
#include "cmsis_os.h"
#include "task.h"
#include "Modbus.h"
#include "gps_app.h"
#include "nmea.h"
void init_read_encoder_task();
typedef enum
{
    BUZZER_OFF = 0,
    BUZZER_7M = 1,
    BUZZER_3M = 2,
} buzzer_mode_t;

#ifdef __cplusplus
}
#endif
