
#include <heartbeat_task.h>
#include <board.h>
#include "iwdg.h"
extern volatile uint8_t g_task_alive_flags;
extern IWDG_HandleTypeDef hiwdg1;
void StartHeartbeatTask(void *argument)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    for (int i = 0; i < 5; i++)
    {
        HAL_IWDG_Refresh(&hiwdg1);
        osDelay(1000);
    }
    uint8_t error_count = 0;
    for (;;)
    {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

        

        if (g_task_alive_flags == TASK_ALL_ALIVE)
        {
            // 喂狗
            HAL_IWDG_Refresh(&hiwdg1);
            g_task_alive_flags = 0;

            if (error_count > 0)
            {
                printf("[INFO] System recovered. Error count reset.-%d\r\n", error_count);
                error_count = 0; // 清空黑本子
            }
        }
        else
        {
            // 没打卡,计数器加 1
            error_count++;
            uint8_t missing_tasks = TASK_ALL_ALIVE & (~g_task_alive_flags);

            // A warning will only start when there are no clock-ins for two consecutive times (within 2 seconds).
            if (error_count == 6)
            {
                printf("[WARNING] Task delayed! Missing: ");
                if (missing_tasks & TASK_AI_SAFY_ALIVE)
                    printf("AI_SAFY ");
                if (missing_tasks & TASK_GPS_ALIVE)
                    printf("GPS ");
                if (missing_tasks & TASK_RELAY_ALIVE)
                    printf("RELAY ");
                if (missing_tasks & TASK_RFID_ALIVE)
                    printf("RFID ");
                printf("\r\n");
            }
            // If you don't clock in within 7 seconds, the watchdog reset system has been triggered
            else if (error_count >= 7)
            {
                printf("[FATAL ERROR] Watchdog will bite NOW! System rebooting...\r\n");
            }

        }
        osDelay(1000);
    }
}
