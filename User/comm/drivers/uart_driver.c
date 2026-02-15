#include "uart_driver.h"

#include <string.h>

#include "usart.h"


#if UART_DRIVER_USE_RINGBUFFER
#include "ringbuffer.h"

// 接收 ringbuffer 实例和缓冲区
static uint8_t      ringbuffer_data[UART_RINGBUFFER_SIZE];
static ringbuffer_t uart_ringbuffer;
#endif

// DMA 接收缓冲区
static uint8_t      dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE];
static uint16_t     last_dma_pos   = 0;
static volatile int tx_in_progress = 0;  // 当前是否有 DMA 发送正在进行

// 接收回调函数
static uart_rx_callback_t user_rx_callback = NULL;

// ==================== 发送队列（环形缓冲区） ====================
#if UART_DRIVER_USE_TX_QUEUE
static uint8_t           tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint16_t tx_wr_idx      = 0;  // 写索引
static volatile uint16_t tx_rd_idx      = 0;  // 读索引
static volatile uint32_t tx_current_len = 0;  // 当前正在发送的数据长度（用于回调推进队列）
#endif

// ==================== 接收处理====================
#if UART_DRIVER_USE_RINGBUFFER
static void process_with_ringbuffer(void)
{
    uint16_t cur_pos = UART_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    if (cur_pos != last_dma_pos) {
        if (cur_pos > last_dma_pos) {
            uint32_t new_data_size = cur_pos - last_dma_pos;
            ringbuffer_write(&uart_ringbuffer, &dma_rx_buffer[last_dma_pos], new_data_size);
        } else {
            uint32_t first_part = UART_DMA_RX_BUFFER_SIZE - last_dma_pos;
            ringbuffer_write(&uart_ringbuffer, &dma_rx_buffer[last_dma_pos], first_part);
            if (cur_pos > 0) {
                ringbuffer_write(&uart_ringbuffer, dma_rx_buffer, cur_pos);
            }
        }
        last_dma_pos = cur_pos;

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
static void process_without_ringbuffer(void)
{
    uint16_t cur_pos = UART_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    if (cur_pos != last_dma_pos) {
        if (cur_pos > last_dma_pos) {
            uint32_t new_data_size = cur_pos - last_dma_pos;
            if (user_rx_callback != NULL) {
                user_rx_callback(&dma_rx_buffer[last_dma_pos], new_data_size);
            }
        } else {
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

// ==================== 发送队列内部函数 ====================
#if UART_DRIVER_USE_TX_QUEUE

/**
 * @brief 向发送队列写入数据（拷贝）
 * @param data 源数据指针
 * @param len  数据长度
 * @return 0:成功, -1:队列空间不足
 */
static int tx_queue_write(const uint8_t* data, uint32_t len)
{
    uint32_t free_space;
    uint32_t wr, rd;

    __disable_irq();  // 进入临界区
    wr = tx_wr_idx;
    rd = tx_rd_idx;

    // 计算剩余空间（环形缓冲区满条件：预留一个单元，避免 wr == rd 歧义）
    if (wr >= rd) {
        free_space = (UART_TX_BUFFER_SIZE - wr) + rd - 1;
    } else {
        free_space = rd - wr - 1;
    }

    if (len > free_space) {
        __enable_irq();
        return -1;  // 队列满
    }

    // 拷贝数据
    for (uint32_t i = 0; i < len; i++) {
        tx_buffer[wr] = data[i];
        wr++;
        if (wr >= UART_TX_BUFFER_SIZE) {
            wr = 0;
        }
    }
    tx_wr_idx = wr;  // 更新写索引
    __enable_irq();
    return 0;
}

/**
 * @brief 获取待发送数据的连续块
 * @param ptr 输出参数，指向连续数据块的起始地址
 * @return 连续数据块的字节数（0 表示无数据）
 */
static uint32_t tx_queue_read_linear(uint8_t** ptr)
{
    uint32_t size;
    __disable_irq();
    if (tx_rd_idx == tx_wr_idx) {
        __enable_irq();
        *ptr = NULL;
        return 0;
    }

    *ptr = &tx_buffer[tx_rd_idx];
    if (tx_wr_idx > tx_rd_idx) {
        size = tx_wr_idx - tx_rd_idx;
    } else {
        size = UART_TX_BUFFER_SIZE - tx_rd_idx;  // 到缓冲区末尾
    }
    __enable_irq();
    return size;
}

/**
 * @brief DMA 发送完成后，推进读指针
 * @param len 已发送的字节数
 */
static void tx_queue_advance(uint32_t len)
{
    __disable_irq();
    tx_rd_idx += len;
    if (tx_rd_idx >= UART_TX_BUFFER_SIZE) {
        tx_rd_idx -= UART_TX_BUFFER_SIZE;
    }
    __enable_irq();
}

#endif /* UART_DRIVER_USE_TX_QUEUE */

// ==================== 公共函数 ====================

int uart_driver_init(uart_rx_callback_t rx_callback)
{
    user_rx_callback = rx_callback;
    last_dma_pos     = 0;
    tx_in_progress   = 0;

#if UART_DRIVER_USE_RINGBUFFER
    ringbuffer_init(&uart_ringbuffer, ringbuffer_data, UART_RINGBUFFER_SIZE);
#endif

#if UART_DRIVER_USE_TX_QUEUE
    tx_wr_idx      = 0;
    tx_rd_idx      = 0;
    tx_current_len = 0;
#endif

    // 启动 DMA 接收（循环模式）
    HAL_UART_Receive_DMA(&huart3, dma_rx_buffer, UART_DMA_RX_BUFFER_SIZE);

    return 0;
}

int uart_driver_send(const uint8_t* data, uint32_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, (uint8_t*)data, len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? (int)len : -1;
}

int uart_driver_send_async(const uint8_t* data, uint32_t len)
{
    if (data == NULL || len == 0) {
        return 0;
    }

#if UART_DRIVER_USE_TX_QUEUE
    // 1. 将数据写入发送队列
    if (tx_queue_write(data, len) != 0) {
        return -1;  // 队列满，发送失败
    }

    // 2. 如果当前没有正在进行的 DMA 传输，则立即启动一包
    if (!tx_in_progress) {
        uint8_t* p;
        uint32_t chunk = tx_queue_read_linear(&p);
        if (chunk > 0) {
            tx_in_progress           = 1;
            tx_current_len           = chunk;  // 记录本次发送长度，供回调使用
            HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart3, p, chunk);
            if (status != HAL_OK) {
                // DMA 启动失败：回退写指针（丢弃刚入队的数据）
                __disable_irq();
                tx_wr_idx -= len;  // 简化回退，实际需循环处理
                if (tx_wr_idx >= UART_TX_BUFFER_SIZE) {
                    tx_wr_idx += UART_TX_BUFFER_SIZE;  // 修正负值情况
                }
                __enable_irq();
                tx_in_progress = 0;
                return -1;
            }
        }
    }
    return 0;  // 数据已入队，发送将在完成后自动继续

#else /* 无发送队列，保持原有逻辑 */
    if (tx_in_progress) {
        return -1;
    }
    tx_in_progress           = 1;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart3, (uint8_t*)data, len);
    return (status == HAL_OK) ? 0 : -1;
#endif
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
#if UART_DRIVER_USE_TX_QUEUE
    // 当没有正在进行的 DMA 传输且发送队列为空时，才算发送完成
    return (!tx_in_progress && (tx_rd_idx == tx_wr_idx));
#else
    return !tx_in_progress;
#endif
}

void uart_driver_tx_complete_callback(void)
{
    // 无论有无队列，先清除发送标志
    tx_in_progress = 0;

#if UART_DRIVER_USE_TX_QUEUE
    // 1. 推进队列读指针（释放已发送的数据）
    if (tx_current_len > 0) {
        tx_queue_advance(tx_current_len);
        tx_current_len = 0;
    }

    // 2. 检查队列中是否还有待发数据
    uint8_t* p;
    uint32_t chunk = tx_queue_read_linear(&p);
    if (chunk > 0) {
        tx_in_progress = 1;
        tx_current_len = chunk;
        HAL_UART_Transmit_DMA(&huart3, p, chunk);
    }
#else
    // 无队列时仅清除标志
#endif
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
