#ifndef __RINGBUFFER_H__
#define __RINGBUFFER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint8_t*          buffer;
    uint32_t          size;
    volatile uint32_t head;   // 写指针
    volatile uint32_t tail;   // 读指针
    volatile uint32_t count;  // 当前数据量
    volatile uint32_t overflow_count;
} ringbuffer_t;

/**
 * @brief 初始化环形缓冲区
 * @param rb 缓冲区结构体指针
 * @param buffer 底层缓冲区指针
 * @param size 缓冲区大小
 */
void ringbuffer_init(ringbuffer_t* rb, uint8_t* buffer, uint32_t size);

/**
 * @brief 写入数据到环形缓冲区
 * @param rb 缓冲区结构体指针
 * @param data 要写入的数据
 * @param len 数据长度
 * @return 实际写入的字节数
 */
uint32_t ringbuffer_write(ringbuffer_t* rb, const uint8_t* data, uint32_t len);

/**
 * @brief 从环形缓冲区读取数据
 * @param rb 缓冲区结构体指针
 * @param buffer 输出缓冲区
 * @param len 请求读取的长度
 * @return 实际读取的字节数
 */
uint32_t ringbuffer_read(ringbuffer_t* rb, uint8_t* buffer, uint32_t len);

/**
 * @brief 查看缓冲区中的数据（不移动读指针）
 * @param rb 缓冲区结构体指针
 * @param buffer 输出缓冲区
 * @param len 请求查看的长度
 * @return 实际查看的字节数
 */
uint32_t ringbuffer_peek(const ringbuffer_t* rb, uint8_t* buffer, uint32_t len);

/**
 * @brief 跳过指定长度的数据
 * @param rb 缓冲区结构体指针
 * @param len 要跳过的长度
 * @return 实际跳过的字节数
 */
uint32_t ringbuffer_skip(ringbuffer_t* rb, uint32_t len);

/**
 * @brief 获取缓冲区中可用数据量
 * @param rb 缓冲区结构体指针
 * @return 可用数据字节数
 */
uint32_t ringbuffer_available(const ringbuffer_t* rb);

/**
 * @brief 获取缓冲区空闲空间
 * @param rb 缓冲区结构体指针
 * @return 空闲空间字节数
 */
uint32_t ringbuffer_free_space(const ringbuffer_t* rb);

/**
 * @brief 检查缓冲区是否为空
 * @param rb 缓冲区结构体指针
 * @return true: 空, false: 非空
 */
bool ringbuffer_is_empty(const ringbuffer_t* rb);

/**
 * @brief 检查缓冲区是否已满
 * @param rb 缓冲区结构体指针
 * @return true: 满, false: 未满
 */
bool ringbuffer_is_full(const ringbuffer_t* rb);

/**
 * @brief 清空缓冲区
 * @param rb 缓冲区结构体指针
 */
void ringbuffer_clear(ringbuffer_t* rb);

/**
 * @brief 获取溢出计数
 * @param rb 缓冲区结构体指针
 * @return 溢出次数
 */
uint32_t ringbuffer_get_overflow_count(const ringbuffer_t* rb);

/**
 * @brief 重置溢出计数
 * @param rb 缓冲区结构体指针
 */
void ringbuffer_reset_overflow_count(ringbuffer_t* rb);

#ifdef __cplusplus
}
#endif

#endif /* __RINGBUFFER_H__ */
