#include "board.h"
#include <stdio.h>
#include "printf_redirect.h"
#include <main.h>
#include "cmsis_os.h"
#include <usart.h>
#include "FreeRTOS.h"
#include "task.h"
#include "read_encoder_task.h"
#include "heartbeat_task.h"

uint32_t get_time_ms(void)
{
#if defined(FREERTOS) || defined(USE_FREERTOS) || defined(configUSE_PREEMPTION)
    // 判断是否在中断中
    if (__get_IPSR() != 0) {
        // 在中断
        return (uint32_t)(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
    } else {
        // 任务上下文
        return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    }
#else
    // 非FreeRTOS环境,HAL_GetTick()本身就是线程安全的，可以在中断服务函数（ISR）中直接调用
    return HAL_GetTick();
#endif
}

osThreadId_t HeartbeatTaskHandle;
const osThreadAttr_t HeartbeatTask_attributes = {
  .name = "HeartbeatTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

void init_app(void)
{
  specify_redirect_uart(&huart1);
  printf("\r\n[INFO] [BOARD] specify redirect printf to huart1\r\n");

  init_read_encoder_task();
  printf("[INFO] [BOARD] ENABLE_ENCODER_TASK\r\n");

  HeartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &HeartbeatTask_attributes);
  printf("[INFO] [BOARD] ENABLE_HEARTBEAT_TASK\r\n");
}
