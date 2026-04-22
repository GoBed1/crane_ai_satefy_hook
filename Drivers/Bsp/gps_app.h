#ifndef GPS_APP_H
#define GPS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "uart_manage.h"
// reg[111]：关机时间，格式高字节=小时/低字节=分钟，例如0x1500=21:00
// reg[112]：开机时间，例如0x0600=06:00
#define STATUS_POWER_OFF_TIME   111
#define STATUS_POWER_ON_TIME    112
#define POWER_OFF_DEFAULT       ((10 << 8) | 22)
#define POWER_ON_DEFAULT        ((10  << 8) | 24)
#define RTC_TIME                114
#define STANDBY_ENABLE            115


extern uart_inferface_t gps_app;

void config_gps_app(void);
void update_gps_app(void);
void rtc_power_init(void);
void rtc_power_schedule_check(void);
uint8_t rtc_is_wakeup_from_standby(void);
void gps_test_nmea_parser(void);
void update_gps_time_loop_test(void);
void print_internal_rtc_time(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_APP_H
