#include "gps_app.h"
// #include "board_manage.h"
#include "nmea.h"

// #define LOGD(...) // SYSLOG_DEBUG("GPS",__VA_ARGS__)
// #define LOGI(...) // SYSLOG_INFO("GPS",__VA_ARGS__)
// #define LOGE(...) // SYSLOG_ERR("GPS",__VA_ARGS__)
#define LOGD(...) printf(__VA_ARGS__)
#define LOGI(...) printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)

#define TSET_GPS_NMEA_PARSER 0

#define WT_RTK_UM982 1
#define WT_GPS_UM626N 2
#define WT_GPS_6N 3
#ifndef GPS_TYPE_STD
#define GPS_TYPE_STD WT_GPS_6N
#endif
// GPS是否已同步（锁星后才允许关机判断）
uint8_t s_gps_synced = 0;
extern RTC_HandleTypeDef hrtc;
extern uint16_t modbus_registers[];
static uint8_t s_test_schedule_configured = 0;
void enter_standby(void);
void set_alarm_b(uint8_t utc_h, uint8_t utc_m);
void gps_sync_rtc_once(void);
void gps_print_nmea_data(const char *tag)
{
    LOGD("[%s] fix=%u sat=%u view=%u mode=%u time=%02u:%02u:%02u date=%04u-%02u-%02u\r\n",
         tag,
         g_nmea_gnss.fix_quality,
         g_nmea_gnss.satellite,
         g_nmea_gnss.satellite_in_view,
         g_nmea_gnss.fix_mode,
         g_nmea_gnss.time_h,
         g_nmea_gnss.time_m,
         g_nmea_gnss.time_s,
         g_nmea_gnss.date_year,
         g_nmea_gnss.date_m,
         g_nmea_gnss.date_d);

    LOGD("[%s] lat=%.6f lon=%.6f alt=%.2f hdop=%.2f pdop=%.2f vdop=%.2f spd=%.2fkn/%.2fkmh cog=%.2f mask=0x%08lX\r\n",
         tag,
         g_nmea_gnss.latitude_deg,
         g_nmea_gnss.longitude_deg,
         g_nmea_gnss.altitude_m,
         g_nmea_gnss.precision_m,
         g_nmea_gnss.pdop_m,
         g_nmea_gnss.vdop_m,
         g_nmea_gnss.speed_knots,
         g_nmea_gnss.speed_kmh,
         g_nmea_gnss.course_deg,
         (unsigned long)g_nmea_gnss.sentence_mask);
}

/*
TEST OUTPUT:
NMEA test[1] OK
NMEA test[2] OK
NMEA test[3] OK
NMEA test[4] OK
NMEA test[5] OK
NMEA test[6] OK
NMEA test[7] OK
NMEA test[8] OK
[TEST] fix=1 sat=8 view=8 mode=3 time=07:32:37 date=2026-03-02
[TEST] lat=22.426970 lon=114.208656 alt=75.88 hdop=1.00 pdop=1.80 vdop=1.50 spd=0.12kn/0.22kmh cog=45.60 mask=0x000000FF
 */
void gps_test_nmea_parser(void)
{
    const char *test_sentences[] = {
        "$GNGGA,073237.00,2225.61814,N,11412.51906,E,1,08,1.7,75.88,M,-2.36,M,,*57\r\n",
        "$GNGLL,2225.61814,N,11412.51906,E,073237.00,A,A*74\r\n",
        "$GNGSA,A,3,08,10,23,16,27,03,14,30,,,,1.8,1.0,1.5*03\r\n",
        "$GNGSV,2,1,08,03,15,120,35,08,45,067,42,10,32,250,40,14,22,300,37*61\r\n",
        "$GNRMC,073237.00,A,2225.61814,N,11412.51906,E,0.12,45.6,020326,,,A*42\r\n",
        "$GNVTG,45.6,T,,M,0.12,N,0.22,K,A*27\r\n",
        "$GNZDA,073237.00,02,03,2026,00,00*7D\r\n",
        "$GNTXT,01,01,02,ANTENNA OK*28\r\n",
    };

    uint32_t ok_count = 0;
    for (uint32_t i = 0; i < (uint32_t)(sizeof(test_sentences) / sizeof(test_sentences[0])); i++)
    {
        const uint8_t *s = (const uint8_t *)test_sentences[i];
        uint16_t n = (uint16_t)strlen(test_sentences[i]);
        int ret = nmea_parse(s, n);
        if (ret == NMEA_OK)
        {
            ok_count++;
            LOGI("NMEA test[%lu] OK\r\n", (unsigned long)(i + 1U));
        }
        else
        {
            LOGE("NMEA test[%lu] FAIL: %d\r\n", (unsigned long)(i + 1U), ret);
        }
    }
    gps_print_nmea_data("TEST");
}

void config_gps_app(void)
{
    (void)uart_manage_enable_dma_recv_by_name("gps");
    osDelay(1000);
    // HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET); // 高电平gps工作
    // osDelay(3000);

#if (GPS_TYPE_STD == WT_RTK_UM982)
    osDelay(10);
    const char cfgmsg_gga[] = "GPGGA COM1 100\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gga, sizeof(cfgmsg_gga) - 1U);
    osDelay(10);
    const char cfgmsg_rmc[] = "GPRMC COM1 1\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_rmc, sizeof(cfgmsg_rmc) - 1U);
    osDelay(10);
    const char cfgmsg_save[] = "SAVECONFIG\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_save, sizeof(cfgmsg_save) - 1U);
    osDelay(10);
#elif (GPS_TYPE_STD == WT_GPS_UM626N)
    // 只启用RMC消息，关闭其他所有消息
    const char cfgmsg_gga[] = "$CFGMSG,0,0,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gga, sizeof(cfgmsg_gga) - 1U);
    osDelay(10);
    const char cfgmsg_gll[] = "$CFGMSG,0,1,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gll, sizeof(cfgmsg_gll) - 1U);
    osDelay(10);
    const char cfgmsg_gsa[] = "$CFGMSG,0,2,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gsa, sizeof(cfgmsg_gsa) - 1U);
    osDelay(10);
    const char cfgmsg_gsv[] = "$CFGMSG,0,3,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gsv, sizeof(cfgmsg_gsv) - 1U);
    osDelay(10);
    const char cfgmsg_rmc[] = "$CFGMSG,0,4,1\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_rmc, sizeof(cfgmsg_rmc) - 1U);
    osDelay(10);
    const char cfgmsg_vtg[] = "$CFGMSG,0,5,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_vtg, sizeof(cfgmsg_vtg) - 1U);
    osDelay(10);
    const char cfgmsg_zda[] = "$CFGMSG,0,6,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_zda, sizeof(cfgmsg_zda) - 1U);
    osDelay(10);
    const char cfgmsg_gst[] = "$CFGMSG,0,7,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gst, sizeof(cfgmsg_gst) - 1U);
    osDelay(10);
#elif (GPS_TYPE_STD == WT_GPS_6N)
    const char cfgmsg_freq[] = "$PCAS03,1,0,0,0,0,0,0,0*03\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_freq, sizeof(cfgmsg_freq) - 1U);
    osDelay(10);
    const char cfgmsg_save[] = "$PCAS00*01\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_save, sizeof(cfgmsg_save) - 1U);
    osDelay(10);
#endif
    HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET); // 高电平gps工作
    osDelay(3000);
}

//=================================模拟 test gps======================================================

void run_10_oclock_standby_test(void)
{
    printf("\r\n========================================\r\n");
    printf("[TEST] 1. INJECTING FAKE GPS TIME (10:00:00 BJ Time)\r\n");

    // 1. 强行覆写 GPS 变量 (北京时间 10:00 = UTC 02:00)
    g_nmea_gnss.fix_quality = 1;
    g_nmea_gnss.time_h = 2; // UTC 2 点
    g_nmea_gnss.time_m = 0; // 0 分
    g_nmea_gnss.time_s = 0; // 0 秒
    g_nmea_gnss.date_year = 2026;
    g_nmea_gnss.date_m = 3;
    g_nmea_gnss.date_d = 12;

    // 2. 触发一次 RTC 同步，让底层的硬件 RTC 跑到 10:00
    gps_sync_rtc_once();
    s_gps_synced = 1;

    // 3. 强行设定关机、开机时间寄存器
    // printf("[TEST] Schedule Configured:\r\n");
    // printf("       Current BJ Time = 10:00\r\n");
    // printf("       Power OFF Time  = 10:01\r\n");
    // printf("       Power ON  Time  = 10:02\r\n");
    // printf("========================================\r\n\r\n");
}

// 循环轮询更新gps时间
void update_gps_time_loop_test(void)
{
    // 手动让时间流逝：每秒给假 GPS 时间加 1 秒
    g_nmea_gnss.time_s++;
    if (g_nmea_gnss.time_s >= 60)
    {
        g_nmea_gnss.time_s = 0;
        g_nmea_gnss.time_m++;
        if (g_nmea_gnss.time_m >= 60)
        {
            g_nmea_gnss.time_m = 0;
            g_nmea_gnss.time_h++;
        }
    }
    printf("[TEST]  GPS time: %02d:%02d:%02d\r\n", g_nmea_gnss.time_h, g_nmea_gnss.time_m, g_nmea_gnss.time_s);
}
//=================================test======================================================

void update_gps_app(void)
{
#if TSET_GPS_NMEA_PARSER
    gps_test_nmea_parser();
    osDelay(1000);
    return;
#endif

    uart_inferface_t *m_obj = uart_manage_get_obj_by_name("gps");
    if (m_obj == NULL)
    {
        LOGE("GPS uart interface not found\r\n");
        return;
    }

    lwrb_sz_t available = lwrb_get_full(&m_obj->process_ring_buffer);
    if (available == 0)
    {
        return;
    }
    uint8_t to_read_buffer[128];
    lwrb_sz_t to_read = (available > sizeof(to_read_buffer)) ? sizeof(to_read_buffer) : available;
    lwrb_sz_t read_size = lwrb_read(&m_obj->process_ring_buffer, to_read_buffer, to_read);

    if (read_size > 0)
    {
        LOGD("Raw GPS data (%lu bytes): %.*s\r\n", (unsigned long)read_size, (int)read_size, (const char *)to_read_buffer);
        int parse_result = nmea_parse(to_read_buffer, (uint16_t)read_size);
        if (parse_result != NMEA_OK && parse_result != NMEA_ERR_NO_FIX)
        {
            LOGE("[PE%d]%.*s\r\n", parse_result, (int)read_size, (const char *)to_read_buffer);
            // gps_test_nmea_parser();
        }
        else
        {
            // RMC消息解析成功，检查是否有有效的时间和日期数据
            // RMC包含：UTC time (field 1) 和 Date (field 7)
            // if (g_nmea_gnss.date_year > 0 && g_nmea_gnss.date_m > 0 && g_nmea_gnss.date_d > 0)
            // {
            if (g_nmea_gnss.time_h > 0 || g_nmea_gnss.time_m > 0 || g_nmea_gnss.time_s > 0)
            {
                LOGD("RMC: parse_result=%d fix=%u time=%02u:%02u:%02u date=%04u-%02u-%02u\r\n",
                     parse_result,
                     g_nmea_gnss.fix_quality,
                     g_nmea_gnss.time_h,
                     g_nmea_gnss.time_m,
                     g_nmea_gnss.time_s,
                     g_nmea_gnss.date_year,
                     g_nmea_gnss.date_m,
                     g_nmea_gnss.date_d);

                gps_sync_rtc_once();
            }
        }
    }
}

// 读取PWR标志位，1=来自待机唤醒，0=正常上电
uint8_t rtc_is_wakeup_from_standby(void)
{
    // 读PWR标志位，1=来自待机唤醒，0=正常上电
    return (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) ? 1 : 0;
}

// 初始化RTC电源管理，设置默认的关机和开机时间
void rtc_power_init(void)
{
    // 解锁备份域访问权限（必须要有，否则无法读取备份寄存器）
    HAL_PWR_EnableBkUpAccess();

    modbus_registers[STATUS_POWER_OFF_TIME] = POWER_OFF_DEFAULT;
    modbus_registers[STATUS_POWER_ON_TIME] = POWER_ON_DEFAULT;

    modbus_registers[STANDBY_ENABLE] = 1; // 默认启用定时待机功能

    if (rtc_is_wakeup_from_standby())
    {
        printf("[PWR] from standby\r\n");
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

        // printf("[PWR] RTC set to on_time UTC %02d:%02d\r\n", on_h_utc, on_m);
    }
    else
    {
        printf("[PWR] cold start\r\n");
        // 检查备份寄存器 RTC_BKP_DR1 中是否有我们写入的标记 0x5AA5
        if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0x5AA5)
        {
            printf("[PWR] RTC time is kept alive by VBAT (Coin Cell)!\r\n");
            // 纽扣电池生效，RTC 时间有效，允许直接进行关机计划检测
            s_gps_synced = 1;
            print_internal_rtc_time();
        }
        else
        {
            printf("[PWR] RTC time invalid or first boot, waiting for GPS lock...\r\n");
            // 时间无效，必须等待 GPS 同步
            s_gps_synced = 0;
        }
    }
}
void print_internal_rtc_time(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // 注意：必须先调用 GetTime，再调用 GetDate！这是 STM32 硬件影子寄存器的要求。
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // 将 RTC 的 UTC 时间转换为北京时间 (UTC+8)
    uint8_t beijing_h = (sTime.Hours + 8) % 24;

    printf("is real write into internal RTC: 20%02u-%02u-%02u %02u:%02u:%02u | Beijing Time: %02u:%02u:%02u\r\n",
           sDate.Year, sDate.Month, sDate.Date,
           sTime.Hours, sTime.Minutes, sTime.Seconds,
           beijing_h, sTime.Minutes, sTime.Seconds);
}
// GPS同步RTC的函数，确保只同步一次
void gps_sync_rtc_once(void)
{
    static uint8_t rtc_synced = 0;
    if (rtc_synced)
    {
        // 已经同步过，跳过
        return;
    }
    // if (g_nmea_gnss.fix_quality < 1)
    //     return; // 还没锁星，跳过

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = g_nmea_gnss.time_h;
    sTime.Minutes = g_nmea_gnss.time_m;
    sTime.Seconds = g_nmea_gnss.time_s;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    // 解锁备份域，并将 0x5AA5 写入备份寄存器 1
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x5AA5);

    print_internal_rtc_time();
    osDelay(100); // 确保RTC寄存器稳定

    rtc_synced = 1;
    s_gps_synced = 1; // 控制关机逻辑，必须锁星后才允许判断
}
// 循环每10s检测
void rtc_power_schedule_check(void)
{
    if (!s_gps_synced)
    {

        return; // GPS未同步，不判断
    }
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // 使用内部 RTC 记录的 UTC 时间转换为北京时间
    uint8_t beijing_h = (sTime.Hours + 8) % 24; // UTC→北京
    uint8_t beijing_m = sTime.Minutes;
    uint16_t now_hhmm = (uint16_t)((beijing_h << 8) | beijing_m);
    // // 直接读GPS解析的UTC时间
    // uint8_t beijing_h = (g_nmea_gnss.time_h + 8) % 24; // UTC→北京
    // uint8_t beijing_m = g_nmea_gnss.time_m;
    // uint16_t now_hhmm = (uint16_t)((beijing_h << 8) | beijing_m);

    uint16_t off_hhmm = modbus_registers[STATUS_POWER_OFF_TIME]; // reg[111]
    uint16_t on_hhmm = modbus_registers[STATUS_POWER_ON_TIME];   // reg[112]

    printf("[PWR] internal RTC beijing %02d:%02d | off=%02d:%02d on=%02d:%02d\r\n",
           beijing_h, beijing_m,
           off_hhmm >> 8, off_hhmm & 0xFF,
           on_hhmm >> 8, on_hhmm & 0xFF);

    // 把当前rtc时间暴露在modbusReg中，方便外部监控
    modbus_registers[RTC_TIME] = now_hhmm;

    if (now_hhmm == off_hhmm && modbus_registers[STANDBY_ENABLE] == 1) // 精确匹配且待机功能启用
    {
        // 关闭LED和喇叭
        modbus_registers[0] = 0;
        modbus_registers[1] = 0;
        modbus_registers[2] = 0;
        modbus_registers[100] = 0;
        modbus_registers[101] = 0;
        uint8_t on_h_utc = ((on_hhmm >> 8) + 24 - 8) % 24; // 北京→UTC
        set_alarm_b(on_h_utc, (uint8_t)(on_hhmm & 0xFF));  // 设RTC闹钟
        enter_standby();                                   // 进入待机，不返回
    }
}

void set_alarm_b(uint8_t utc_h, uint8_t utc_m)
{
    // 开启 PWR 时钟 + 备份域访问（H7 必须）
    // __HAL_RCC_PWR_CLK_ENABLE();

    HAL_PWR_EnableBkUpAccess();

    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B); // 先关旧闹钟

    RTC_AlarmTypeDef sAlarm = {0};
    sAlarm.AlarmTime.Hours = utc_h;
    sAlarm.AlarmTime.Minutes = utc_m;
    sAlarm.AlarmTime.Seconds = 0;

    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    // 时分触发，忽略秒和星期
    sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_SECONDS;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = 1;
    sAlarm.Alarm = RTC_ALARM_B;

    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        printf("[RTC] Alarm B SET FAILED!\r\n");
        return;
    }

    __HAL_RTC_ALARM_EXTI_ENABLE_IT();
    __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();

    printf("[RTC] Alarm B: UTC %02d:%02d\r\n", utc_h, utc_m);
}

// 进入待机，不返回
void enter_standby(void)
{
    // ================== 安全校验防线 ==================
    // 1. Check if the backup domain data is still there?
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x5AA5)
    {
        printf("[PWR-ERR] Backup domain invalid! Magic number lost.\r\n");
        s_gps_synced = 0; // 取消同步标志
        return;           // 拒绝休眠，退回去继续等 GPS 信号
    }

    // 2. Check the system's global synchronization flag bit (double insurance)
    if (!s_gps_synced)
    {
        printf("[PWR-ERR] System not synced with GPS/VBAT. Abort standby.\r\n");
        return;
    }
    printf("[PWR] enter standby mode...\r\n");
    osDelay(200);
    HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET); // 高电平gps工作
    osDelay(100);
    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRBF); // 清闹钟标志
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();                 // ← 新增：清EXTI挂起位
                                                       // 3. 清H7的唤醒标志（WUF1~WUF6，全部清掉）
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP1);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP3);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP4);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP5);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP6);
    // 4. 清待机/停止标志（H7用CSSF）
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB); // 实际写PWR->CPUCR的CSSF位
    HAL_PWR_EnterSTANDBYMode();
    // 不会执行到这里
}
