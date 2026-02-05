#include "ringbuffer.h"

#include <string.h>

void ringbuffer_init(ringbuffer_t* rb, uint8_t* buffer, uint32_t size)
{
    rb->buffer         = buffer;
    rb->size           = size;
    rb->head           = 0;
    rb->tail           = 0;
    rb->count          = 0;
    rb->overflow_count = 0;
}

uint32_t ringbuffer_write(ringbuffer_t* rb, const uint8_t* data, uint32_t len)
{
    if (len == 0 || data == NULL) {
        return 0;
    }

    // 计算实际可写入的长度
    uint32_t free_space = ringbuffer_free_space(rb);
    if (len > free_space) {
        rb->overflow_count++;
        len = free_space;  // 只写入能写入的部分
    }

    if (len == 0) {
        return 0;
    }

    // 写入数据
    uint32_t first_part = rb->size - rb->head;
    if (len <= first_part) {
        // 可以一次性写入
        memcpy(&rb->buffer[rb->head], data, len);
        rb->head += len;
        if (rb->head >= rb->size) {
            rb->head = 0;
        }
    } else {
        // 需要分两次写入（环绕）
        memcpy(&rb->buffer[rb->head], data, first_part);
        memcpy(rb->buffer, data + first_part, len - first_part);
        rb->head = len - first_part;
    }

    rb->count += len;
    return len;
}

uint32_t ringbuffer_read(ringbuffer_t* rb, uint8_t* buffer, uint32_t len)
{
    if (len == 0 || buffer == NULL) {
        return 0;
    }

    // 计算实际可读取的长度
    uint32_t available = ringbuffer_available(rb);
    if (len > available) {
        len = available;
    }

    if (len == 0) {
        return 0;
    }

    // 读取数据
    uint32_t first_part = rb->size - rb->tail;
    if (len <= first_part) {
        // 可以一次性读取
        memcpy(buffer, &rb->buffer[rb->tail], len);
        rb->tail += len;
        if (rb->tail >= rb->size) {
            rb->tail = 0;
        }
    } else {
        // 需要分两次读取（环绕）
        memcpy(buffer, &rb->buffer[rb->tail], first_part);
        memcpy(buffer + first_part, rb->buffer, len - first_part);
        rb->tail = len - first_part;
    }

    rb->count -= len;
    return len;
}

uint32_t ringbuffer_peek(const ringbuffer_t* rb, uint8_t* buffer, uint32_t len)
{
    if (len == 0 || buffer == NULL) {
        return 0;
    }

    // 计算实际可查看的长度
    uint32_t available = ringbuffer_available(rb);
    if (len > available) {
        len = available;
    }

    if (len == 0) {
        return 0;
    }

    // 查看数据（不移动指针）
    uint32_t first_part = rb->size - rb->tail;
    if (len <= first_part) {
        memcpy(buffer, &rb->buffer[rb->tail], len);
    } else {
        memcpy(buffer, &rb->buffer[rb->tail], first_part);
        memcpy(buffer + first_part, rb->buffer, len - first_part);
    }

    return len;
}

uint32_t ringbuffer_skip(ringbuffer_t* rb, uint32_t len)
{
    if (len == 0) {
        return 0;
    }

    // 计算实际可跳过的长度
    uint32_t available = ringbuffer_available(rb);
    if (len > available) {
        len = available;
    }

    if (len == 0) {
        return 0;
    }

    // 移动读指针
    rb->tail = (rb->tail + len) % rb->size;
    rb->count -= len;

    return len;
}

uint32_t ringbuffer_available(const ringbuffer_t* rb)
{
    return rb->count;
}

uint32_t ringbuffer_free_space(const ringbuffer_t* rb)
{
    return rb->size - rb->count;
}

bool ringbuffer_is_empty(const ringbuffer_t* rb)
{
    return rb->count == 0;
}

bool ringbuffer_is_full(const ringbuffer_t* rb)
{
    return rb->count >= rb->size;
}

void ringbuffer_clear(ringbuffer_t* rb)
{
    rb->head  = 0;
    rb->tail  = 0;
    rb->count = 0;
}

uint32_t ringbuffer_get_overflow_count(const ringbuffer_t* rb)
{
    return rb->overflow_count;
}

void ringbuffer_reset_overflow_count(ringbuffer_t* rb)
{
    rb->overflow_count = 0;
}
