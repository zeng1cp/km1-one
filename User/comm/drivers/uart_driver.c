#include "uart_driver.h"

#include <string.h>

#include "usart.h"

#if UART_DRIVER_USE_RINGBUFFER
#include "ringbuffer.h"

// ringbuffer实例和缓冲区
static uint8_t      ringbuffer_data[UART_RINGBUFFER_SIZE];
static ringbuffer_t uart_ringbuffer;
#endif

// DMA接收缓冲区
static uint8_t      dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
static uint16_t     last_dma_pos   = 0;
static volatile int tx_in_progress = 0;

// 接收回调函数
static uart_rx_callback_t user_rx_callback = NULL;

#if UART_DRIVER_USE_RINGBUFFER
// 使用ringbuffer时的内部处理函数
static void process_with_ringbuffer(void)
{
    uint16_t cur_pos = UART_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

    if (cur_pos != last_dma_pos) {
        if (cur_pos > last_dma_pos) {
            // 线性数据
            uint32_t new_data_size = cur_pos - last_dma_pos;
            ringbuffer_write(&uart_ringbuffer, &dma_rx_buffer[last_dma_pos], new_data_size);
        } else {
            // 环形缓冲环绕
            uint32_t first_part = UART_DMA_RX_BUFFER_SIZE - last_dma_pos;
            ringbuffer_write(&uart_ringbuffer, &dma_rx_buffer[last_dma_pos], first_part);

            if (cur_pos > 0) {
                ringbuffer_write(&uart_ringbuffer, dma_rx_buffer, cur_pos);
            }
        }

        last_dma_pos = cur_pos;

        // 如果用户设置了回调，将数据传递给回调
        if (user_rx_callback != NULL) {
            uint8_t  temp_buffer[64];
            uint32_t read_size;

            while ((read_size = ringbuffer_read(&uart_ringbuffer, temp_buffer, sizeof(temp_buffer)))
                   > 0) {
                user_rx_callback(temp_buffer, read_size);
            }
        }
    }
}
#else
// 不使用ringbuffer时的内部处理函数
static void process_without_ringbuffer(void)
{
    uint16_t cur_pos = UART_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);

    if (cur_pos != last_dma_pos) {
        if (cur_pos > last_dma_pos) {
            // 线性数据
            uint32_t new_data_size = cur_pos - last_dma_pos;
            if (user_rx_callback != NULL) {
                user_rx_callback(&dma_rx_buffer[last_dma_pos], new_data_size);
            }
        } else {
            // 环形缓冲环绕
            uint32_t first_part = UART_DMA_RX_BUFFER_SIZE - last_dma_pos;
            if (user_rx_callback != NULL) {
                user_rx_callback(&dma_rx_buffer[last_dma_pos], first_part);
            }

            if (cur_pos > 0 && user_rx_callback != NULL) {
                user_rx_callback(dma_rx_buffer, cur_pos);
            }
        }

        last_dma_pos = cur_pos;
    }
}
#endif

// ==================== 公共函数 ====================
int uart_driver_init(uart_rx_callback_t rx_callback)
{
    user_rx_callback = rx_callback;
    last_dma_pos     = 0;
    tx_in_progress   = 0;

#if UART_DRIVER_USE_RINGBUFFER
    // 初始化ringbuffer
    ringbuffer_init(&uart_ringbuffer, ringbuffer_data, UART_RINGBUFFER_SIZE);
#endif

    // 启动DMA接收（循环模式）
    HAL_UART_Receive_DMA(&huart3, dma_rx_buffer, UART_DMA_RX_BUFFER_SIZE);

    return 0;  // 假设总是成功
}

int uart_driver_send(const uint8_t* data, uint32_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }

    // 阻塞式发送
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, (uint8_t*)data, len, HAL_MAX_DELAY);

    return (status == HAL_OK) ? len : -1;
}

int uart_driver_send_async(const uint8_t* data, uint32_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }

    if (tx_in_progress) {
        return -1;  // 发送忙
    }

    tx_in_progress           = 1;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart3, (uint8_t*)data, len);

    return (status == HAL_OK) ? 0 : -1;
}

void uart_driver_poll(void)
{
#if UART_DRIVER_USE_RINGBUFFER
    process_with_ringbuffer();
#else
    process_without_ringbuffer();
#endif
}

int uart_driver_is_tx_done(void)
{
    return !tx_in_progress;
}

#if UART_DRIVER_USE_RINGBUFFER
uint32_t uart_driver_read_buffer(uint8_t* buffer, uint32_t size)
{
    return ringbuffer_read(&uart_ringbuffer, buffer, size);
}

uint32_t uart_driver_get_available(void)
{
    return ringbuffer_available(&uart_ringbuffer);
}

void uart_driver_clear_buffer(void)
{
    ringbuffer_clear(&uart_ringbuffer);
}
#endif

void uart_driver_tx_complete_callback(void)
{
    tx_in_progress = 0;
}
