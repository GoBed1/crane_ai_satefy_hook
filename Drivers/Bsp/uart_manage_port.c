/*
 * 文件用途：Echo - 直接回调直通模式（Callback-Direct Mode）
 *
 * 如何使用：
 * 1) 将本文件重命名为 uart_manage_port.c 并加入工程编译。
 * 2) 在系统初始化阶段调用 init_uart_manage()。
 * 3) 保证 HAL_UARTEx_RxEventCallback/HAL_UART_TxCpltCallback 未被其他文件重复定义覆盖。
 *
 * 预期效果：
 * - 收到串口数据后，uart_manage 在 RxEvent 中直接调用 echo_callback。
 * - echo_callback 内立即调用 uart_manage_dma_send_by_name("echo", ...) 回发。
 * - 上位机向 USART1 发送 "abc"，应快速收到 "abc" 回显。
 */
/* port.c */
#include "uart_manage.h"
#include "Modbus.h"
extern RFIDClient RFID_client;
extern EventGroupHandle_t eg; // 初始化事件组为NULL

/* DMA buffer placement */
#if defined(__GNUC__)
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define DMA_BUFFER
#endif

#define LOGD(...) printf(__VA_ARGS__)
#define LOGI(...) printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
static uint8_t uart1_recv_buff[256U] DMA_BUFFER;
static uint8_t uart1_send_buff[256U] DMA_BUFFER;
static uint8_t uart1_send_fifo_buff[256U] DMA_BUFFER;
static uint8_t uart1_process_buff[256U * 4U] DMA_BUFFER;

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
static uint8_t uart5_recv_buff[256U] DMA_BUFFER;
static uint8_t uart5_send_buff[256U] DMA_BUFFER;
static uint8_t uart5_send_fifo_buff[256U] DMA_BUFFER;
static uint8_t uart5_process_buff[256U * 4U] DMA_BUFFER;

static uint32_t echo_callback(uint8_t *buf, uint16_t len)
{
  (void)uart_manage_dma_send_by_name("echo", buf, len);
  return 0U;
}

const uart_inferface_t uart_manage_table[] = {
  {
    .name = "echo",
    .uart_h = &huart1,
    .dma_h = &hdma_usart1_rx,
    .recv_buffer = uart1_recv_buff,
    .recv_buffer_size = sizeof(uart1_recv_buff),
    .process_buffer = uart1_process_buff,
    .process_buffer_size = sizeof(uart1_process_buff),
    .recv_callback = echo_callback,// callback_direct_mode
    .send_buffer = uart1_send_buff,
    .send_buffer_size = sizeof(uart1_send_buff),
    .send_fifo_buffer = uart1_send_fifo_buff,
    .send_fifo_size = sizeof(uart1_send_fifo_buff),
    .send_callback = NULL,
  },
  {
    .name = "gps",
    .uart_h = &huart5,
    .dma_h = &hdma_uart5_rx,
    .recv_buffer = uart5_recv_buff,
    .recv_buffer_size = sizeof(uart5_recv_buff),
    .process_buffer = uart5_process_buff,
    .process_buffer_size = sizeof(uart5_process_buff),
    .recv_callback = NULL,// ring_task_mode
    .send_buffer = uart5_send_buff,
    .send_buffer_size = sizeof(uart5_send_buff),
    .send_fifo_buffer = uart5_send_fifo_buff,
    .send_fifo_size = sizeof(uart5_send_fifo_buff),
    .send_callback = NULL,
  },
};

#define uart_manage_table_size \
  ((uint16_t)(sizeof(uart_manage_table) / sizeof(uart_manage_table[0])))

void init_uart_manage(void)
{
  (void)uart_manage_init_table(uart_manage_table, uart_manage_table_size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  (void)uart_manage_enable_dma_recv(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_manage_send_completed_hook(huart);

   /* Modbus RTU TX callback BEGIN */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int i;
  for (i = 0; i < numberHandlers; i++)
  {
    if (mHandlers[i]->port == huart)
    {
      // notify the end of TX
      xTaskNotifyFromISR(mHandlers[i]->myTaskModbusAHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
      break;
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void *)huart;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /* Modbus RTU RX callback BEGIN */
  int i;
  for (i = 0; i < numberHandlers; i++)
  {
    if (mHandlers[i]->port == huart)
    {

      if (mHandlers[i]->xTypeHW == USART_HW)
      {
        RingAdd(&mHandlers[i]->xBufferRX, mHandlers[i]->dataRX);
        HAL_UART_Receive_IT(mHandlers[i]->port, &mHandlers[i]->dataRX, 1);
        xTimerResetFromISR(mHandlers[i]->xTimerT35, &xHigherPriorityTaskWoken);
      }
      break;
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  uart_inferface_t *m_obj = uart_manage_get_obj(huart);

  if (m_obj != NULL)
  {
    if (size > 0U)
    {
      (void)uart_manage_recv_idle_hook(m_obj, INTERRUPT_TYPE_UART, size);
    }
    (void)uart_manage_enable_dma_recv(huart);
  }

 for (int i = 0; i < numberHandlers; i++)
    {
      if (mHandlers[i]->port == huart)
      {

        if (mHandlers[i]->xTypeHW == USART_HW_DMA)
        {
          while (HAL_UARTEx_ReceiveToIdle_DMA(mHandlers[i]->port, mHandlers[i]->xBufferRX.uxBuffer, MAX_BUFFER) != HAL_OK)
          {
            HAL_UART_DMAStop(mHandlers[i]->port);
          }
          __HAL_DMA_DISABLE_IT(mHandlers[i]->port->hdmarx, DMA_IT_HT); // we don't need half-transfer interrupt
        }

        break;
      }
    }

   BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (huart->Instance == huart2.Instance)
  {
    if (RFID_client.rx_buf[0] == 0x1B && RFID_client.rx_buf[1] == 0x39 && RFID_client.rx_buf[2] == 0x01) // RFID从机
    {
      if (size > 0)
      {
        memcpy(RFID_client.Rx_RFID_buf, RFID_client.rx_buf, size);
        RFID_client.Rx_RFID_len = (uint8_t)size;
      }
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RFID_client.rx_buf, (uint16_t)sizeof(RFID_client.rx_buf));
      xEventGroupSetBitsFromISR(eg, EVENT_RFID_RX, &xHigherPriorityTaskWoken);
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  (void *)huart;
}



