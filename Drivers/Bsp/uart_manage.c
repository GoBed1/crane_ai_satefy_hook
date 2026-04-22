#include "uart_manage.h"

#include <stdint.h>
#include <string.h>

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32h7xx_hal_uart.h"

#define LOGD(...) printf(__VA_ARGS__)
#define LOGI(...) printf(__VA_ARGS__)
#define LOGE(...) printf(__VA_ARGS__)

static uart_inferface_t uart_manage[UART_MANAGE_MAX_OBJECTS] = {0};

static inline uintptr_t dma_align_down_32(uintptr_t addr)
{
  return addr & ~(uintptr_t)31U;
}

static inline uintptr_t dma_align_up_32(uintptr_t addr)
{
  return (addr + 31U) & ~(uintptr_t)31U;
}

static inline void dma_clean_cache_by_addr(const void *addr, uint32_t len)
{
// #if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
//   uintptr_t start = dma_align_down_32((uintptr_t)addr);
//   uintptr_t end   = dma_align_up_32((uintptr_t)addr + len);
//   SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
// #else
  (void)addr;
  (void)len;
// #endif
}

static inline void dma_invalidate_cache_by_addr(const void *addr, uint32_t len)
{
// #if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
//   uintptr_t start = dma_align_down_32((uintptr_t)addr);
//   uintptr_t end   = dma_align_up_32((uintptr_t)addr + len);
//   SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
// #else
  (void)addr;
  (void)len;
// #endif
}

static int uart_manage_find_slot_by_huart(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return -1;
  }

  for (uint32_t i = 0U; i < UART_MANAGE_MAX_OBJECTS; ++i)
  {
    if ((uart_manage[i].used != 0U) && (uart_manage[i].uart_h == huart))
    {
      return (int)i;
    }
  }

  return -1;
}

static int uart_manage_find_free_slot(void)
{
  for (uint32_t i = 0U; i < UART_MANAGE_MAX_OBJECTS; ++i)
  {
    if (uart_manage[i].used == 0U)
    {
      return (int)i;
    }
  }

  return -1;
}

uart_inferface_t *uart_manage_get_obj(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return NULL;
  }

  for (uint32_t i = 0U; i < UART_MANAGE_MAX_OBJECTS; ++i)
  {
    if ((uart_manage[i].used != 0U) && (uart_manage[i].uart_h == huart))
    {
      return &uart_manage[i];
    }
  }

  return NULL;
}

uart_inferface_t *uart_manage_get_obj_by_name(const char *name)
{
  if (name == NULL)
  {
    return NULL;
  }

  for (uint32_t i = 0U; i < UART_MANAGE_MAX_OBJECTS; ++i)
  {
    if ((uart_manage[i].used != 0U) && (strcmp(uart_manage[i].name, name) == 0))
    {
      return &uart_manage[i];
    }
  }

  return NULL;
}

int uart_manage_register_interface(uart_inferface_t *m_obj)
{
  if (m_obj == NULL || m_obj->uart_h == NULL)
  {
    return -1;
  }

  int slot = uart_manage_find_slot_by_huart(m_obj->uart_h);
  if (slot < 0)
  {
    slot = uart_manage_find_free_slot();
  }

  if (slot < 0)
  {
    return -1;
  }

  uart_manage[slot] = *m_obj;
  uart_manage[slot].used = 1U;
  uart_manage[slot].idx = (uint8_t)slot;
  if (uart_manage[slot].send_fifo_buffer != NULL && uart_manage[slot].send_fifo_size > 0U)
  {
    fifo_s_init(&uart_manage[slot].send_fifo, uart_manage[slot].send_fifo_buffer, uart_manage[slot].send_fifo_size);
  }
  uart_manage[slot].is_sending = 0U;

  if ((uart_manage[slot].process_buffer != NULL) && (uart_manage[slot].process_buffer_size > 0U))
  {
    (void)lwrb_init(&uart_manage[slot].process_ring_buffer, uart_manage[slot].process_buffer, uart_manage[slot].process_buffer_size);
  }
  
  return 0;
}

int uart_manage_set_recv_callback(UART_HandleTypeDef *huart, interface_recv_fn_t recv_callback)
{
  uart_inferface_t *obj = uart_manage_get_obj(huart);
  if (obj == NULL)
  {
    return -1;
  }

  obj->recv_callback = recv_callback;
  return 0;
}

int uart_manage_init_table(const uart_inferface_t *table, uint16_t table_size)
{
  if (table == NULL || table_size == 0U || table_size > UART_MANAGE_MAX_OBJECTS)
  {
    LOGE("Invalid uart manage table\r\n");
    return -1;
  }

  for (uint16_t i = 0U; i < table_size; ++i)
  {
    if (uart_manage_register_interface((uart_inferface_t *)&table[i]) != 0)
    {
      LOGE("Failed to register uart interface: %s\r\n", table[i].name);
      return -1;
    }
    LOGI("Registered uart interface: %s\r\n", table[i].name);
    (void)uart_manage_enable_dma_recv(table[i].uart_h);
  }

  return 0;
}

void uart_manage_enable_dma_recv(UART_HandleTypeDef *huart)
{
  uart_inferface_t *m_obj = uart_manage_get_obj(huart);

  if (m_obj == NULL || m_obj->dma_h == NULL || m_obj->uart_h == NULL)
  {
    return;
  }

  m_obj->uart_h->Instance->ICR = USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF | USART_ICR_IDLECF;
  (void)m_obj->uart_h->Instance->RDR;

  HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(m_obj->uart_h, m_obj->recv_buffer, m_obj->recv_buffer_size);
  if (st == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(m_obj->dma_h, DMA_IT_HT);
    return;
  }

  if (st == HAL_BUSY)
  {
    HAL_Delay(100);
  }

  (void)HAL_UART_DMAStop(m_obj->uart_h);
  m_obj->uart_h->Instance->ICR = USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF | USART_ICR_IDLECF;
  (void)m_obj->uart_h->Instance->RDR;
  st = HAL_UARTEx_ReceiveToIdle_DMA(m_obj->uart_h, m_obj->recv_buffer, m_obj->recv_buffer_size);
  if (st == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(m_obj->dma_h, DMA_IT_HT);
  }else{
    printf("double enable idle dma recv failed[%X]\r\n",m_obj->uart_h);
  }
}

void uart_manage_enable_dma_recv_by_name(const char *name)
{
  uart_inferface_t *m_obj = uart_manage_get_obj_by_name(name);

  if (m_obj == NULL || m_obj->dma_h == NULL || m_obj->uart_h == NULL)
  {
    return;
  }

  m_obj->uart_h->Instance->ICR = USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF | USART_ICR_IDLECF;
  (void)m_obj->uart_h->Instance->RDR;

  HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(m_obj->uart_h, m_obj->recv_buffer, m_obj->recv_buffer_size);
  if (st == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(m_obj->dma_h, DMA_IT_HT);
    return;
  }

  if (st == HAL_BUSY)
  {
    HAL_Delay(100);
  }

  (void)HAL_UART_DMAStop(m_obj->uart_h);
  m_obj->uart_h->Instance->ICR = USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF | USART_ICR_IDLECF;
  (void)m_obj->uart_h->Instance->RDR;
  st = HAL_UARTEx_ReceiveToIdle_DMA(m_obj->uart_h, m_obj->recv_buffer, m_obj->recv_buffer_size);
  if (st == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(m_obj->dma_h, DMA_IT_HT);
  }else{
    printf("double enable idle dma recv failed[%X]\r\n",m_obj->uart_h);
  }
}

static int uart_manage_dma_send_impl(uart_inferface_t *m_obj, uint8_t *buf, uint16_t len)
{
  if(m_obj == NULL || buf == NULL || len == 0U)
  {
    return -1;
  }

  uint16_t to_send_len;
  uint16_t to_tx_fifo_len;

  if (m_obj->is_sending == 0U)
  {
    if (len < m_obj->send_buffer_size)
    {
      to_send_len = len;
      to_tx_fifo_len = 0;
    }
    else if (len < m_obj->send_buffer_size + m_obj->send_fifo_size)
    {
      to_send_len = m_obj->send_buffer_size;
      to_tx_fifo_len = len - m_obj->send_buffer_size;
    }
    else
    {
      to_send_len = m_obj->send_buffer_size;
      to_tx_fifo_len = m_obj->send_fifo_size;
    }
  }
  else
  {
    if (len < m_obj->send_fifo_size)
    {
      to_send_len = 0;
      to_tx_fifo_len = len;
    }
    else
    {
      to_send_len = 0;
      to_tx_fifo_len = m_obj->send_fifo_size;
    }
  }

  if (to_send_len > 0)
  {
    memcpy(m_obj->send_buffer, buf, to_send_len);
    dma_clean_cache_by_addr(m_obj->send_buffer, to_send_len);
    m_obj->is_sending = 1U;
    HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->send_buffer, to_send_len);
  }
  if (to_tx_fifo_len > 0)
  {
    uint8_t put_len;
    put_len = fifo_s_puts(&m_obj->send_fifo, (char *)(buf) + to_send_len, to_tx_fifo_len);
    if (put_len != to_tx_fifo_len)
    {
      return -1;
    }
  }
  return 0;
}

int uart_manage_dma_send(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t len)
{
  uart_inferface_t *m_obj = uart_manage_get_obj(huart);
  return uart_manage_dma_send_impl(m_obj, buf, len);
}

int uart_manage_dma_send_by_name(const char *name, uint8_t *buf, uint16_t len)
{
  uart_inferface_t *m_obj = uart_manage_get_obj_by_name(name);
  return uart_manage_dma_send_impl(m_obj, buf, len);
}

void uart_manage_send_completed_hook(UART_HandleTypeDef *huart)
{
  uart_inferface_t *m_obj = uart_manage_get_obj(huart);

    if(m_obj == NULL)
    {
        return;
    }

    uint16_t fifo_data_num = 0;
    uint16_t send_num = 0;

    fifo_data_num = m_obj->send_fifo.used_num;

    if (fifo_data_num != 0)
    {
      if (fifo_data_num < m_obj->send_buffer_size)
        {
            send_num = fifo_data_num;
        }
        else
        {
        send_num = m_obj->send_buffer_size;
        }
      fifo_s_gets(&m_obj->send_fifo, (char *)m_obj->send_buffer, send_num);
      dma_clean_cache_by_addr(m_obj->send_buffer, send_num);
      m_obj->is_sending = 1U;
      HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->send_buffer, send_num);
    }
    else
    {
      m_obj->is_sending = 0U;
    }
    return;
}

int uart_manage_write_to_recv_ring(uart_inferface_t *m_obj, uint8_t *buf, uint16_t len)
{
  if (m_obj == NULL || buf == NULL || len == 0U)
  {
    return -1;
  }
  
  lwrb_sz_t to_write_len = (lwrb_sz_t)len;
  lwrb_sz_t free_len = lwrb_get_free(&m_obj->process_ring_buffer);
  if (to_write_len > free_len)
  {
    to_write_len = free_len;
  }

  if (lwrb_write(&m_obj->process_ring_buffer, buf, to_write_len) != to_write_len)
  {
    return -1;
  }
  return (int)to_write_len;
}

void uart_manage_recv_idle_hook(uart_inferface_t *m_obj, interrput_type int_type, uint16_t size)
{
  
  if ((m_obj == NULL) || (size == 0U) || (size > m_obj->recv_buffer_size))
  {
    return;
  }
  
  (void)int_type;

  dma_invalidate_cache_by_addr(m_obj->recv_buffer, size);
  
  if(m_obj->recv_callback != NULL)
  {
    m_obj->recv_callback(m_obj->recv_buffer, size);
    return;
  }

  uart_manage_write_to_recv_ring(m_obj, m_obj->recv_buffer, size);
}









