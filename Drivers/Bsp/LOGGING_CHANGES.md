# crane_ai_satefy_hook 日志系统改造说明

本文件概述此次对日志系统的主要改动、使用方法与初始化时序，帮助你快速理解和使用新的异步 DMA 日志框架。

## 目标
- 减少 printf 在任务中的阻塞和堆栈压力，避免串口逐字节输出导致的卡顿或 HardFault。
- 支持在多任务环境下的日志整洁性（可选互斥保护）。
- 在需要时以最低侵入方式将 `printf` 切换到异步 DMA 输出。

## 关键改动一览

- 新增异步日志模块：`Drivers/Bsp/async_logger.h/.c`
  - 基于环形缓冲区 + DMA 发送 + 日志任务的方案，实现非阻塞写入。
  - 通过 `HAL_UART_Transmit_DMA` 进行批量发送，降低 CPU 占用和任务堆栈消耗。
  - 提供 ISR 钩子 `async_logger_on_tx_cplt_from_isr`，由 `HAL_UART_TxCpltCallback` 中调用以推进发送队列。

- 优化 `printf` 重定向：`Drivers/Bsp/printf_redirect.h/.c`
  - 可选互斥：提供 `printf_create_mutex / printf_set_mutex / printf_use_mutex`，避免多任务同时 `printf` 带来的串扰。
  - 可选异步：提供 `printf_redirect_use_async_logger(int enable)`，启动后 `_write` 会路由到异步日志模块，实现非阻塞写入。
  - 统一 newlib syscall 的 errno/返回值行为：非法 fd → `EBADF`；底层 HAL 错误 → `EIO`；失败返回 `-1`。

- 回调衔接：`Drivers/MODBUS-LIB/UARTCallback.c`
  - 在 `HAL_UART_TxCpltCallback` 内追加调用 `async_logger_on_tx_cplt_from_isr(huart)`，与 Modbus 处理兼容，不破坏原有逻辑。

- DMA/DMAMUX 兼容性修复：`Core/Src/dma.c`
  - `__HAL_RCC_DMAMUX1_CLK_ENABLE()` 在 H7 HAL 中并不总是存在，现用 `#ifdef` 包裹以保证可移植与链接通过。

## 模块接口

### 1) 异步日志模块（`async_logger.h`）
- `int async_logger_init(UART_HandleTypeDef *huart);`
  - 绑定用于 DMA 发送的 UART 句柄，创建内部对象。
- `int async_logger_start(void);`
  - 启动日志任务（默认优先级：低于 normal，栈 1024）。
- `size_t async_logger_write(const uint8_t *data, size_t len, uint32_t timeout_ms, int drop_if_full);`
  - 非阻塞入队；`drop_if_full=1` 时空间不足直接丢弃；否则等待直至超时。
- `void async_logger_on_tx_cplt_from_isr(UART_HandleTypeDef *huart);`
  - 从 `HAL_UART_TxCpltCallback` 调用的 ISR 钩子，用于通知一次 DMA 发送完成。
- 可选：`void async_logger_flush(uint32_t timeout_ms);`、`int async_logger_is_running(void);`

### 2) printf 重定向（`printf_redirect.h`）
- `void specify_redirect_uart(UART_HandleTypeDef *huart);`
  - 设置 printf 使用的 UART 句柄，并关闭 stdout 的行缓冲（立即发送）。
- `osMutexId_t printf_create_mutex(void);`
- `void printf_set_mutex(osMutexId_t mutex);`
- `void printf_use_mutex(int enable);`
  - 三者组合用于启用/配置 `printf` 的互斥保护；默认关闭以节省资源。
- `void printf_redirect_use_async_logger(int enable);`
  - 启用后，`_write` 调用会改为投递到 `async_logger_write`；否则使用 `HAL_UART_Transmit` 阻塞发送。

## 推荐初始化时序

1. 在 `MX_USART1_UART_Init()` 之后（FreeRTOS 启动前）
   - `specify_redirect_uart(&huart1);`
   - 可选：创建并启用互斥
     - `printf_create_mutex();`
     - `printf_use_mutex(1);`
   - 初始化异步日志模块
     - `async_logger_init(&huart1);`

2. 在 `osKernelInitialize()`/`MX_FREERTOS_Init()` 之后（调度器启动前后均可，但确保 RTOS 就绪）
   - `async_logger_start();`
   - `printf_redirect_use_async_logger(1);`  // 将 printf 路由到异步日志

3. 在 `HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)` 中追加：
   - `async_logger_on_tx_cplt_from_isr(huart);`

> 以上步骤已经在当前工程中完成对接：`main.c` 完成初始化，`UARTCallback.c` 完成 ISR 钩子。

## 运行期行为

- 当异步模式开启且运行时：
  - `printf` → `_write` → `async_logger_write`，快速将数据入环形缓冲区，后台任务按块发送 DMA。
  - 若缓冲区满且 `drop_if_full=1`：日志可能被丢弃以避免阻塞（当前策略）。
- 当异步模式未开启：
  - `printf` 采用 `HAL_UART_Transmit` 阻塞发送；若启用互斥，可避免多任务输出交叉。

## 注意事项与排错

- 确保 USART1 的 DMA TX 已正确链接（`__HAL_LINKDMA`）与中断打开，当前工程已核对无误。
- 若使用 DMAMUX 同步或请求发生器等高级功能，可考虑实现 `DMAMUX1_OVR_IRQHandler` 并在其中调用 `HAL_DMAEx_MUX_IRQHandler`；常规外设 DMA 请求不需要。
- 若日志量非常大，建议：
  - 适度增大 `ASYNC_LOGGER_RING_SIZE`；
  - 在关键路径减少格式化耗时；
  - 只在必要的信息点输出。

## 变更清单（文件级）

- 新增：
  - `Drivers/Bsp/async_logger.h` / `Drivers/Bsp/async_logger.c`：异步 DMA 日志模块。
- 修改：
  - `Drivers/Bsp/printf_redirect.h` / `Drivers/Bsp/printf_redirect.c`：互斥支持、异步路由、errno 语义规范。
  - `Drivers/MODBUS-LIB/UARTCallback.c`：在 `HAL_UART_TxCpltCallback` 中追加异步日志的 ISR 通知。
  - `Core/Src/dma.c`：DMAMUX 时钟宏加 `#ifdef` 保护，解决链接错误并保持可移植性。
- 相关：
  - `Core/Src/main.c`：在合适的时序中完成上述初始化与切换（已接到 `huart1`）。

---
如需将日志切回阻塞模式：在运行时调用 `printf_redirect_use_async_logger(0);` 即可（也可直接不启动异步模块）。
