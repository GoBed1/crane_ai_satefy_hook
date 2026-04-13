#include "read_encoder_task.h"
// #include "encoder_forward_app.h"

// #define LOGD(...)  printf(__VA_ARGS__)
#define LOGD(...) //printf(__VA_ARGS__)

#define LOGI(...) printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)

 volatile uint8_t g_task_alive_flags;

#define CMD_LED_SWITCH 0
#define STATUS_LED_SWITCH 100
// 喇叭
#define CMD_BUZZER_7m 1
#define CMD_BUZZER_3m 2
// #define CMD_BUZZER_stop 3

#define STATUS_BUZZER 101
#define STATUS_BMS_BATTERY 102
#define STATUS_HEART_BEAT 104
#define STATUS_BMS_REMAIN_DISCHARGE_TIME 105
#define STATUS_WORK_MODE 106
#define STATUS_BMS_IS_charge 107
#define STATUS_BMS_REMAIN_CHARGE_TIME 108
#define STATUS_BMS_TOTAL_VOLTAGE 109
#define STATUS_BMS_TOTAL_CURRENT 110
// #define STATUS_POWER_OFF_TIME   111
// #define STATUS_POWER_ON_TIME    112
#define REG_ERROR_CODE 113 // 错误码

// ========== 内部函数：大端拼接 ==========

// 放电时间全局变量
uint32_t discharge_samples[30]; // 采样20个
uint8_t discharge_idx = 0;
uint8_t discharge_count = 0;
// 充电时间全局变量
uint32_t charge_samples[30]; // 采样20个
uint8_t charge_idx = 0;
uint8_t charge_count = 0;
// extern nmea_gnss_t g_nmea_gnss;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart8;
extern RTC_HandleTypeDef hrtc;
// Slave全局变量
static modbusHandler_t encoder_forward_server;
#define REGS_TOTAL_NUM 256
uint16_t modbus_registers[REGS_TOTAL_NUM] = {0};
uint16_t modbus_input_registers[REGS_TOTAL_NUM] = {0};
uint16_t now_volume;      // 假设音量寄存器地址为103
uint16_t last_volume = 0; // 确保开机第一次循环必定能进if分支

void RFID_master_thread(void *argument);
void RFID_OnFrame(RFIDClient *c, const uint8_t *frm, uint16_t len);
void RFID_CheckOffline(RFIDClient *c);
void RFID_WriteToModbusRegs(RFIDClient *c);

osThreadId_t ai_safy_slave_handle;
const osThreadAttr_t ai_safy_slave_attributes = {
    .name = "AISafySlave",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t ai_safy_master_handle;
const osThreadAttr_t ai_safy_master_attributes = {
    .name = "AISafyMaster",
    .stack_size = 1024 * 6,
    .priority = (osPriority_t)osPriorityNormal1,
};
// RFID
osThreadId_t RFID_master_handle;
const osThreadAttr_t RFID_master_attributes = {
    .name = "RFIDMaster",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};
// gps待机线程
osThreadId_t gps_standby_handle;
const osThreadAttr_t gps_standby_attributes = {
    .name = "GPSStandby",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};
// 心跳检测任务
osThreadId_t relay_heartbeat_handle;
const osThreadAttr_t relay_heartbeat_attributes = {
    .name = "RelayHeartbeat",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

uint8_t first_findVolume = 1;
EventGroupHandle_t eg = NULL; // 初始化事件组为NULL
void EventGroupCreate_Init(void)
{
    if (eg == NULL)
    {
        eg = xEventGroupCreate();
    }
}
// ========== 内部函数：大端拼接 ==========
static uint32_t be_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           ((uint32_t)p[3] << 0);
}

// ========== 内部函数：解析RFID帧 ==========
static int parse_frame(const uint8_t *frm, uint16_t len,
                       uint8_t *out_rssi, uint8_t *out_rfid_battery, uint32_t *out_uid)
{
    if (len < 14)
        return 0;
    if (frm[0] != 0x1B || frm[1] != 0x39 || frm[2] != 0x01)
        return 0;

    *out_rssi = frm[7];
    *out_rfid_battery = frm[9];
    *out_uid = be_u32(&frm[10]);

    return 1;
}

// ========== 内部函数：查找UID ==========
static int find_uid(RFIDClient *c, uint32_t uid)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) && c->tags[i].uid == uid)
        {
            return i;
        }
    }
    return -1;
}

// ========== 内部函数：分配空闲槽位 ==========
static int alloc_slot(RFIDClient *c)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) == 0)
        {
            return i;
        }
    }
    return -1;
}

// rfid相关3个函数的实现
// 1.写入Modbus寄存器
void RFID_WriteToModbusRegs(RFIDClient *c)
{
    // 更新位图到 modbus_registers[3] 的低8位
    modbus_registers[REG_RFID_VALID] =
        (modbus_registers[REG_RFID_VALID] & 0xFF00) | c->valid_bitmap;

    // 写8组数据到 modbus_registers[4~27]
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        uint16_t base = REG_RFID_BASE + i * 3;

        if (c->valid_bitmap & (1U << i))
        {
            // 标签有效时写入数据
            modbus_registers[base + 0] = (uint16_t)(c->tags[i].uid >> 16);
            modbus_registers[base + 1] = (uint16_t)(c->tags[i].uid & 0xFFFF);
            modbus_registers[base + 2] = ((uint16_t)c->tags[i].rssi << 8) | c->tags[i].rfid_battery;
        }
        else
        {
            // 标签无效时清零
            modbus_registers[base + 0] = 0;
            modbus_registers[base + 1] = 0;
            modbus_registers[base + 2] = 0;
        }
    }
}
// 2.检查离线
void RFID_CheckOffline(RFIDClient *c)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t timeout = pdMS_TO_TICKS(RFID_OFFLINE_MS);

    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) == 0)
            continue;

        if ((uint32_t)(now - c->tags[i].last_seen_tick) > timeout)
        {
            c->valid_bitmap &= ~(1U << i); // 标记标签无效
            LOGD("RFID offline: idx=%d, UID=0x%08X\n",
                 i, (unsigned int)c->tags[i].uid);
            memset(&c->tags[i], 0, sizeof(c->tags[i])); // 清除标签结构体数据
        }
    }
}
// 3. 收到帧后更新
void RFID_OnFrame(RFIDClient *c, const uint8_t *frm, uint16_t len)
{
    uint8_t rssi, rfid_battery;
    uint32_t uid;

    if (!parse_frame(frm, len, &rssi, &rfid_battery, &uid))
    {
        LOGD("RFID parse fail\n");
        return;
    }

    uint32_t now = xTaskGetTickCount();

    int idx = find_uid(c, uid);
    if (idx < 0)
    {
        idx = alloc_slot(c);
        if (idx < 0)
        {
            LOGD("RFID slots full, UID=0x%08X\n", (unsigned int)uid);
            return;
        }
        c->valid_bitmap |= (1U << idx);
        c->tags[idx].uid = uid;
        LOGD("RFID new tag: idx=%d, UID=0x%08X\n", idx, (unsigned int)uid);
    }

    c->tags[idx].rssi = rssi;
    c->tags[idx].rfid_battery = rfid_battery;
    c->tags[idx].last_seen_tick = now;

    LOGD("RFID update: idx=%d, UID=0x%08X, RSSI=%d, rfid_battery=%d\n",
         idx, (unsigned int)uid, rssi, rfid_battery);
}

void init_ai_safy_slave(void)
{
    // 初始化寄存器数组
    memset(modbus_registers, 0, sizeof(modbus_registers));

    // HACK 设置系统信息
    modbus_registers[REG_ERROR_CODE] = 0x0000; // 清零错误码/在线状态
    // 配置MODBUS Slave处理器
    encoder_forward_server.uModbusType = MB_SLAVE;
    encoder_forward_server.u8id = FORWARD_SLAVE_ADDR; // Slave ID=3
    encoder_forward_server.port = &huart7;
    encoder_forward_server.EN_Port = NULL; // 无RS485控制引脚
    encoder_forward_server.EN_Pin = 0;
    encoder_forward_server.u16regs = modbus_registers;            // 保持寄存器
    encoder_forward_server.u16inputregs = modbus_input_registers; // 输入寄存器
    encoder_forward_server.u16regsize = REGS_TOTAL_NUM;
    encoder_forward_server.u16timeOut = 1000; // 1秒超时
    encoder_forward_server.xTypeHW = USART_HW;
    LOGI("MODBUS-RTU Slave ID: %d\r\n", FORWARD_SLAVE_ADDR);

    // 初始化MODBUS Slave
    ModbusInit(&encoder_forward_server);
    ModbusStart(&encoder_forward_server);

    LOGI("Register range: 0x0000-0x%04X (%d registers)\r\n",
         REGS_TOTAL_NUM - 1, REGS_TOTAL_NUM);
}

modbusHandler_t bms_sound_light_app;
modbusHandler_t rfid_app;

uint16_t bms_results[2] = {0};
uint16_t remainDischargeTime_results[2] = {0};

uint16_t firstVolume_results[2] = {0};
modbus_t telegram[8];

modbus_t telegram2[3];

// HACK
uint16_t modbus_master_buf[128] = {0};
uint16_t modbus_master_buf2[128] = {0};

// 喇叭逻辑处理函数
void buzzer_logic(void)
{
    // 算目标模式：3m 优先于 7m，3m覆盖7m，二者都为0时关闭喇叭。
    // 关联：2=3m, 1=7m, 0=OFF
    uint16_t target_mode = 0;

    if (modbus_registers[CMD_BUZZER_3m] == 1)
    {
        target_mode = 2;
    }
    else if (modbus_registers[CMD_BUZZER_7m] == 1)
    {
        target_mode = 1;
    }
    else
    {
        target_mode = 0;
    }

    // 只在变化时执行
    // static buzzer_mode_t current = BUZZER_OFF;
    // 心跳掉电时，STATUS_BUZZER 会被清 0
    uint16_t current_mode = modbus_registers[STATUS_BUZZER];

    if (target_mode == current_mode)
        return;
    uint32_t err = 0;
    
if (target_mode == 2) // 执行开启 3M
    {
        telegram[1].u16RegAdd = 0x0008;
        telegram[1].u16reg[0] = 0x0009;
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        
        if (err != OP_OK_QUERY) {
            LOGD("BUZZER_3M write fail, retrying...\n");
        } else {
            LOGD("BUZZER_3M write success\n");
            modbus_registers[STATUS_BUZZER] = 2; // 【关键】发送成功才将状态登记为 2
        }
    }
    else if (target_mode == 1) // 执行开启 7M
    {
        telegram[1].u16RegAdd = 0x0008;
        telegram[1].u16reg[0] = 0x0008;
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        
        if (err != OP_OK_QUERY) {
            LOGD("BUZZER_7M write fail, retrying...\n");
        } else {
            LOGD("BUZZER_7M write success\n");
            modbus_registers[STATUS_BUZZER] = 1; // 发送成功才将状态登记为 1
        }
    }
    else // 执行关闭 (OFF)
    {
        telegram[1].u16RegAdd = 0x000E;
        telegram[1].u16reg[0] = 0x0000;
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        
        if (err != OP_OK_QUERY) {
            LOGD("BUZZER_SOUND_STOP write fail, retrying...\n");
        } else {
            LOGD("BUZZER_SOUND_STOP write success\n");
            modbus_registers[STATUS_BUZZER] = 0; // 发送成功才将状态清 0
        }
    }
}

void modbus_TxData_logic(void)
{
    uint16_t cmd_led_switch = modbus_registers[CMD_LED_SWITCH];
    // 读取寄存器0的值，例如
    // 灯打开或关闭命令
    if (cmd_led_switch == 1 && modbus_registers[STATUS_LED_SWITCH] == 0)
    {
        LOGD(" LED on \n");
        telegram[1].u16RegAdd = 0x00C2;
        telegram[1].u16reg[0] = 0x0051; // 慢闪，爆闪改为61
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (err== OP_OK_QUERY)
        {
            LOGD("LED on write success : %d \n", err);
            // 更新状态寄存器regs[100]且清除命令寄存器regs[0]
            modbus_registers[STATUS_LED_SWITCH] = 1;
        }
        else
        {
            LOGD("LED on write fail : %d \n", err);
            
        }

        
    }

    if (cmd_led_switch == 0 && modbus_registers[STATUS_LED_SWITCH] == 1)
    {

        // YX95R_LIGHT_OFF;
        LOGD(" LED off \n");
        telegram[1].u16RegAdd = 0x00C2;
        telegram[1].u16reg[0] = 0x0060;
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        if (err== OP_OK_QUERY)
        {
            LOGD("LED off write success : %d \n", err);
            //成功后更新状态寄存器regs[100]且清除命令寄存器regs[0]
            modbus_registers[STATUS_LED_SWITCH] = 0;
        }
        else
        {
            LOGD("LED off write fail : %d \n", err);
            
        }
        // 通知RX任务：我发送了命令，你可以等待响应了！
        // xEventGroupSetBits(eg, EVENT_CMD_SENT);
    }

    // 喇叭逻辑处理
    buzzer_logic();
}

void ai_safy_master_thread(void *argument)

{
    memset(modbus_master_buf, 0, sizeof(modbus_master_buf));

    bms_sound_light_app.uModbusType = MB_MASTER;
    bms_sound_light_app.port = &huart8;
    bms_sound_light_app.u8id = 0; // For master it must be 0
    bms_sound_light_app.u16timeOut = 500;
    bms_sound_light_app.EN_Port = NULL;
    bms_sound_light_app.EN_Pin = 0;
    bms_sound_light_app.u16regs = modbus_master_buf;
    bms_sound_light_app.u16regsize = sizeof(modbus_master_buf) / sizeof(modbus_master_buf[0]);
    bms_sound_light_app.xTypeHW = USART_HW;
    ModbusInit(&bms_sound_light_app);
    ModbusStart(&bms_sound_light_app);

    LOGD("bms sound light modbus master start \n");

    // read bms
    telegram[0].u8id = 4;
    telegram[0].u8fct = MB_FC_READ_REGISTERS;
    telegram[0].u16RegAdd = 0x0000;
    telegram[0].u16CoilsNo = 1;
    telegram[0].u16reg = bms_results;

    // write light control
    telegram[1].u8id = 1;
    telegram[1].u8fct = MB_FC_WRITE_REGISTER;
    telegram[1].u16CoilsNo = 1;

    // write buzzer control
    telegram[2].u8id = 2;
    telegram[2].u8fct = MB_FC_WRITE_REGISTER;
    telegram[2].u16CoilsNo = 1;

    // read remain discharge time
    telegram[3].u8id = 4;
    telegram[3].u8fct = MB_FC_READ_REGISTERS;
    telegram[3].u16RegAdd = 0x0007;
    telegram[3].u16CoilsNo = 1;
    telegram[3].u16reg = remainDischargeTime_results;

    // read total voltage
    telegram[4].u8id = 4;
    telegram[4].u8fct = MB_FC_READ_REGISTERS;
    telegram[4].u16RegAdd = 0x0002;
    telegram[4].u16CoilsNo = 1;

    // is_charging
    telegram[5].u8id = 4;
    telegram[5].u8fct = MB_FC_READ_REGISTERS;
    telegram[5].u16RegAdd = 0x000B;
    telegram[5].u16CoilsNo = 1;

    // charge remain time
    telegram[6].u8id = 4;
    telegram[6].u8fct = MB_FC_READ_REGISTERS;
    telegram[6].u16RegAdd = 0x0008;
    telegram[6].u16CoilsNo = 1;

    // read total current
    telegram[7].u8id = 4;
    telegram[7].u8fct = MB_FC_READ_REGISTERS;
    telegram[7].u16RegAdd = 0x0001;
    telegram[7].u16CoilsNo = 1;

    TickType_t last_10s = xTaskGetTickCount();
    TickType_t last_500ms = xTaskGetTickCount();
    TickType_t last_5s = xTaskGetTickCount();
    // 心跳机制计时器
    TickType_t last_heartbeat = xTaskGetTickCount();

    for (;;)
    {
        g_task_alive_flags |= TASK_AI_SAFY_ALIVE;
        // 设置音量
        now_volume = modbus_registers[103];

        // 读取音量寄存器
        if (now_volume != last_volume)
        { // 如果音量有变化
            telegram[1].u16RegAdd = 0x0006;
            telegram[1].u16reg[0] = now_volume;
            ModbusQuery(&bms_sound_light_app, telegram[1]); // 调整喇叭的实际输出音量。
            uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            if (err == OP_OK_QUERY)
            {
                LOGD("BUZZER_VOLUME write success, lastvolume=%d , nowVolume=%d,modbusReg[103]:%d\n", last_volume, now_volume, modbus_registers[103]);
                last_volume = now_volume; // 更新上一次的音量记录以供下次比较使用。
            }
            else
            {
                LOGD("BUZZER_VOLUME write fail : %d \n", err);
            }
        }

        // 每 500ms 轮询一次led & buzzer
        if (xTaskGetTickCount() - last_500ms >= pdMS_TO_TICKS(500))
        {
            last_500ms += pdMS_TO_TICKS(500);
            modbus_TxData_logic();

            // 每500ms采样一次放电时间
            ModbusQuery(&bms_sound_light_app, telegram[3]);
            int err1 = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            if (err1 != OP_OK_QUERY)
            {
                LOGD("bms dischange time modbus master read fail : %d \n", err1);
            }
            else
            {
                uint16_t val = telegram[3].u16reg[0];
                if (val != 0xFFFF)
                {
                    discharge_samples[discharge_idx % 20] = val; // 放入缓冲容器中
                    discharge_idx++;
                    if (discharge_count < 20)
                        discharge_count++;
                }
            }

            // 每500ms采样一次充电时间
            ModbusQuery(&bms_sound_light_app, telegram[6]);
            int err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            if (err != OP_OK_QUERY)
            {
                LOGD("bms charge time modbus master read fail : %d \n", err);
            }
            else
            {
                uint16_t val = telegram[6].u16reg[0]; // 假设充电数据在相同位置，若实际位置不同需调整
                if (val != 0xFFFF)
                {
                    charge_samples[charge_idx % 20] = val; // 放入充电缓冲容器中
                    charge_idx++;
                    if (charge_count < 20)
                        charge_count++;
                }
            }
        }

        // 每 10s 执行一次（读取电量/电流） &&   每10s执行一次平均值放入寄存器（剩余放电时间）
        if (xTaskGetTickCount() - last_10s >= pdMS_TO_TICKS(10000))
        {
            last_10s += pdMS_TO_TICKS(10000);
            ModbusQuery(&bms_sound_light_app, telegram[0]);
            int err1 = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            if (err1 != OP_OK_QUERY)
            {
                LOGD("bms led sound modbus master read fail : %d \n", err1);
            }
            else
            {
                modbus_registers[STATUS_BMS_BATTERY] = telegram[0].u16reg[0];
                LOGD("bms led sound modbus master read success,Battery = %d\n", telegram[0].u16reg[0]);
                // LOGD("work mode = %d\n", modbus_registers[STATUS_WORK_MODE]);
            }
            // 是否在充电状态：0-否，1-是

            // 总电压读取
            ModbusQuery(&bms_sound_light_app, telegram[4]);
            int err2 = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            if (err2 != OP_OK_QUERY)
            {
                LOGD("bms total voltage modbus master read fail : %d \n", err2);
            }
            else
            {
                modbus_registers[STATUS_BMS_TOTAL_VOLTAGE] = telegram[4].u16reg[0];
                LOGD("bms total voltage = %d\n", telegram[4].u16reg[0]);
                // LOGD("work mode = %d\n", modbus_registers[STATUS_BMS_TOTAL_VOLTAGE]);
            }

            // //总电流读取 && 判断是否充电状态0-否，1-是
            ModbusQuery(&bms_sound_light_app, telegram[7]);
            int err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
            if (err != OP_OK_QUERY)
            {
                LOGD("bms total current modbus master read fail : %d \n", err);
            }
            else
            {
                int16_t val = (int16_t)telegram[7].u16reg[0];
                // 判断是否充电状态
                if (val < 0)
                {
                    modbus_registers[STATUS_BMS_TOTAL_CURRENT] = (uint16_t)(-val);
                    modbus_registers[STATUS_BMS_IS_charge] = 0; // 放电状态
                }
                else if (val > 0)
                {
                    modbus_registers[STATUS_BMS_TOTAL_CURRENT] = (uint16_t)val;
                    modbus_registers[STATUS_BMS_IS_charge] = 1; // 充电状态
                }
                else
                {
                    modbus_registers[STATUS_BMS_IS_charge] = 2; // 无充电或放电状态
                }
                LOGD("bms charge status = %d\n", modbus_registers[STATUS_BMS_IS_charge]);
                LOGD("bms total current111 = %d===%d\n", modbus_registers[STATUS_BMS_TOTAL_CURRENT], val);
            }

            // 每500ms采样一次放电时间，每10s执行一次平均值放入寄存器（剩余放电时间）
            if (discharge_count > 0)
            {
                uint32_t sum = 0;
                for (uint8_t i = 0; i < discharge_count; i++)
                {
                    sum += discharge_samples[i];
                }
                uint16_t avg = (uint16_t)(sum / discharge_count);
                modbus_registers[STATUS_BMS_REMAIN_DISCHARGE_TIME] = avg;
                LOGD("remain discharge time avg (5s) = %d min\n", modbus_registers[STATUS_BMS_REMAIN_DISCHARGE_TIME]);
            }
            else
            {
                modbus_registers[STATUS_BMS_REMAIN_DISCHARGE_TIME] = 0xFFFF;
                LOGD("remain discharge time: no valid sample\n");
            }

            // 清空缓冲
            memset(discharge_samples, 0, sizeof(discharge_samples));
            discharge_idx = 0;
            discharge_count = 0;

            // 每500ms采样一次充电时间，每10s执行一次平均值放入寄存器（剩余充电时间）
            if (charge_count > 0)
            {
                uint32_t sum = 0;
                for (uint8_t i = 0; i < charge_count; i++)
                {
                    sum += charge_samples[i];
                }
                uint16_t avg = (uint16_t)(sum / charge_count);
                modbus_registers[STATUS_BMS_REMAIN_CHARGE_TIME] = avg;
                LOGD("remain charge time avg (5s) = %d min\n", modbus_registers[STATUS_BMS_REMAIN_CHARGE_TIME]);
            }
            else
            {
                modbus_registers[STATUS_BMS_REMAIN_CHARGE_TIME] = 0xFFFF;
                LOGD("remain charge time: no valid sample\n");
            }

            // 清空缓冲
            memset(charge_samples, 0, sizeof(charge_samples));
            charge_idx = 0;
            charge_count = 0;
        }

        // 判断工作状态
        if ((modbus_registers[STATUS_LED_SWITCH] == 1 && modbus_registers[STATUS_BUZZER] == 1) ||
            (modbus_registers[STATUS_LED_SWITCH] == 1 && modbus_registers[STATUS_BUZZER] == 0) ||
            (modbus_registers[STATUS_LED_SWITCH] == 0 && modbus_registers[STATUS_BUZZER] == 1))
        {
            if (modbus_registers[STATUS_BMS_BATTERY] <= 2000)
            {
                modbus_registers[STATUS_WORK_MODE] = 2; // 工作状态为2，即低电量警告模式
            }
            else
            {
                modbus_registers[STATUS_WORK_MODE] = 1; // 工作状态为1，即正常工作模式
            }
        }
        else
        {
            if (modbus_registers[STATUS_BMS_BATTERY] <= 2000)
            {
                modbus_registers[STATUS_WORK_MODE] = 2; // 工作状态为2，即低电量警告模式
            }
            else
            {

                modbus_registers[STATUS_WORK_MODE] = 0; // 工作状态为0，即非正常工作模式
            }
        }

        osDelay(500);
    }
}
// modbus_t telegram;
RFIDClient RFID_client;
void RFID_master_thread(void *argument)
{

    EventGroupCreate_Init();
    TickType_t last_check = xTaskGetTickCount();
    for (;;)
    {
        g_task_alive_flags |= TASK_RFID_ALIVE;
        EventBits_t uxBits = xEventGroupWaitBits(
            eg,            // 事件组
            EVENT_RFID_RX, // 等待这个事件
            pdTRUE,        // 自动清除标志
            pdFALSE,       // 不需要等待所有位
            pdMS_TO_TICKS(500)   // 有看门狗的超时时间，这里是500ms
        );
        if ((uxBits & EVENT_RFID_RX) != 0)
        {
            LOGD("Recevice rfid: ");
            LOGD("modbus_reg[3] = %04X (", modbus_registers[3]);
            // 打印二进制格式
            for (int i = 15; i >= 0; i--)
            {
                LOGD("%d", (modbus_registers[3] >> i) & 1);
                if (i % 4 == 0 && i != 0)
                    LOGD(" "); // 每4位加空格分隔
            }
            // LOGD(")\n");
            // for (int i = 0; i < 12; i++)
            // {
            //     LOGD("%04X ", modbus_registers[i + 4]);
            //     if (i % 3 == 0)
            //         LOGD("\n");
            // }
            // LOGD("\n");
            RFID_OnFrame(&RFID_client,
                         RFID_client.Rx_RFID_buf,
                         RFID_client.Rx_RFID_len);

            // API 3: 立即写入Modbus寄存器
            RFID_WriteToModbusRegs(&RFID_client);
        }

        // 每1秒检查一次离线
        if ((TickType_t)(xTaskGetTickCount() - last_check) >= pdMS_TO_TICKS(5000))
        {
            last_check = xTaskGetTickCount();
            RFID_CheckOffline(&RFID_client);
            RFID_WriteToModbusRegs(&RFID_client);
        }

        osDelay(1000);
    }
}
// GPS待机线程
void gps_standby_thread(void *argument)
{

    config_gps_app();
    rtc_power_init();
    // run_10_oclock_standby_test();

    for (;;)
    {
        g_task_alive_flags |= TASK_GPS_ALIVE;
        // 每1s轮询一次GPS数据
        // 每1s轮询一次GPS数据
        update_gps_app();
        // // 检测是否进入待机状态
        // run_10_oclock_standby_test();
        // update_gps_time_loop();
        // print_internal_rtc_time();

        rtc_power_schedule_check();

        HAL_GPIO_TogglePin(GPIOD, H_B_LED_Pin);
        // HAL_GPIO_TogglePin(RELAY_1_PIN_GPIO_Port, RELAY_1_PIN_Pin);

        osDelay(1000);
    }
}
// 继电器心跳检测线程
void relay_heartbeat_thread(void *argument)
{
    // 初始化上次心跳值和接收时间
    uint16_t last_heartbeat_val = modbus_registers[STATUS_HEART_BEAT];
    TickType_t recv_heartbeat_time = xTaskGetTickCount();
    // 刚开机时默认是有电状态，1代表有电，0代表断电
    uint8_t relay_is_on = 1;
    for (;;)
    {
        g_task_alive_flags |= TASK_RELAY_ALIVE;
        uint16_t current_heartbeat = modbus_registers[STATUS_HEART_BEAT];

        // 1. 检查心跳是否有变化
        if (current_heartbeat != last_heartbeat_val)
        {
            last_heartbeat_val = current_heartbeat;
            recv_heartbeat_time = xTaskGetTickCount(); // 更新最后一次收到心跳的时间
            if (relay_is_on == 0)
            {
                // 收到心跳，继电器引脚置为1 (上电)
                HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_SET);
                last_volume = 0xFFFF; // 音量更新,主循环会更新
                relay_is_on = 1;      // 标记为有电状态
                modbus_registers[STATUS_LED_SWITCH]=0;// 灯关闭,如果指令寄存器=1，主循环会处理的

            }
            else
            {
                // 收到心跳，继电器引脚置为1 (上电)
                HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_SET);
                LOGD("[Heartbeat] Host active. Relay 2 SET to 1. Reg[104]=%d\r\n", current_heartbeat);
            }
        }
        else
        {
            // 2. 如果没有变化，检查是否超时 1 分钟 (60000 毫秒)
            if ((xTaskGetTickCount() - recv_heartbeat_time) > pdMS_TO_TICKS(60 * 1000) && relay_is_on == 1)
            {
                // 声光警报关闭
                //  关闭LED和喇叭
                modbus_registers[CMD_LED_SWITCH] = 0;
                modbus_registers[CMD_BUZZER_7m] = 0;
                modbus_registers[CMD_BUZZER_3m] = 0;
                modbus_registers[STATUS_LED_SWITCH] = 0;
                modbus_registers[STATUS_BUZZER] = 0;

                relay_is_on = 0; // 标记为断电状态

                // 超时1分钟，继电器引脚置为0 (下电)
                HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_RESET);
                LOGD("[Heartbeat] Timeout (>1 min). Relay 2 SET to 0.\r\n");
            }
        }

        // 每秒轮询一次
        osDelay(1000);
    }
}

void init_read_encoder_task()
{
    // HAL_GPIO_TogglePin(GPIOD, GPS_EN_Pin);
    init_ai_safy_slave();
    // 串口初始化
    init_uart_manage();

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RFID_client.rx_buf, (uint16_t)sizeof(RFID_client.rx_buf));
    // modbus_registers[0] = 0;
    // 初始音量调为最大
    modbus_registers[103] = 0x001E; // 30的十六进制
    // now_volume = 0x1E;
    // 关机&开机时间设置为每天的21:00-次日6:00
    // modbus_registers[STATUS_POWER_OFF_TIME] = (21 << 8) | 0; // 21:00
    // modbus_registers[STATUS_POWER_ON_TIME] = (6 << 8) | 0;   // 06:00

    // HAL_GPIO_TogglePin(GPIOD, GPS_EN_Pin);
    HAL_GPIO_WritePin(RELAY_1_PIN_GPIO_Port, RELAY_1_PIN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_SET);

    relay_heartbeat_handle = osThreadNew(relay_heartbeat_thread, NULL, &relay_heartbeat_attributes);
    ai_safy_master_handle = osThreadNew(ai_safy_master_thread, NULL, &ai_safy_master_attributes);
    RFID_master_handle = osThreadNew(RFID_master_thread, NULL, &RFID_master_attributes);
    gps_standby_handle = osThreadNew(gps_standby_thread, NULL, &gps_standby_attributes);
}
