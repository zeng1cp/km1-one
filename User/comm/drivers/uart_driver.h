#ifndef __UART_DRIVER_H__
#define __UART_DRIVER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

// ==================== 配置选项 ====================
// 外部可以在comm_config.h中定义这些选项来覆盖默认值

#ifndef UART_DRIVER_USE_RINGBUFFER
#define UART_DRIVER_USE_RINGBUFFER 0  // 默认不使用ringbuffer
#endif

#ifndef UART_DMA_RX_BUFFER_SIZE
#define UART_DMA_RX_BUFFER_SIZE 1024  // DMA缓冲区大小
#endif

#if UART_DRIVER_USE_RINGBUFFER
#ifndef UART_RINGBUFFER_SIZE
#define UART_RINGBUFFER_SIZE 512  // 软件ringbuffer大小
#endif
#endif
// ================================================

/**
 * @brief UART数据接收回调函数
 * @param data 接收到的数据
 * @param len 数据长度
 */
typedef void (*uart_rx_callback_t)(const uint8_t* data, uint32_t len);

/**
 * @brief 初始化UART驱动
 * @param rx_callback 数据接收回调（可以为NULL）
 * @return 0:成功, 其他:失败
 */
int uart_driver_init(uart_rx_callback_t rx_callback);

/**
 * @brief 发送数据（阻塞式）
 * @param data 要发送的数据
 * @param len 数据长度
 * @return 发送的字节数
 */
int uart_driver_send(const uint8_t* data, uint32_t len);

/**
 * @brief 发送数据（异步，非阻塞）
 * @param data 要发送的数据
 * @param len 数据长度
 * @return 0:成功, 其他:失败
 */
int uart_driver_send_async(const uint8_t* data, uint32_t len);

/**
 * @brief 轮询处理（在主循环中调用）
 */
void uart_driver_poll(void);

/**
 * @brief 检查发送是否完成
 * @return 1:完成, 0:进行中
 */
int uart_driver_is_tx_done(void);

#if UART_DRIVER_USE_RINGBUFFER
/**
 * @brief 从内部缓冲区读取数据（仅当启用ringbuffer时可用）
 * @param buffer 输出缓冲区
 * @param size 缓冲区大小
 * @return 实际读取的字节数
 */
uint32_t uart_driver_read_buffer(uint8_t* buffer, uint32_t size);

/**
 * @brief 获取内部缓冲区中可用数据量（仅当启用ringbuffer时可用）
 * @return 可用数据字节数
 */
uint32_t uart_driver_get_available(void);

/**
 * @brief 清空内部缓冲区（仅当启用ringbuffer时可用）
 */
void uart_driver_clear_buffer(void);
#endif

/**
 * @brief DMA传输完成回调（由HAL_UART_TxCpltCallback调用）
 */
void uart_driver_tx_complete_callback(void);

#ifdef __cplusplus
}
#endif

#endif /* __UART_DRIVER_H__ */
