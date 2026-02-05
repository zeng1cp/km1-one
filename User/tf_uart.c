#include "tf_uart.h"

#include <stdio.h>
#include <string.h>

#include "servo.h"
#include "servo_action.h"
#include "usart.h"

// === DMA 接收相关 ===
#define UART_DMA_BUF_SIZE 128
static uint8_t   uart_dma_buf[UART_DMA_BUF_SIZE];
static uint16_t  last_dma_pos = 0;
static uint8_t   temp_buf[UART_DMA_BUF_SIZE];  // 临时缓冲区
// === TinyFrame 实例 ===
static TinyFrame tf;

void tf_uart_init(void)
{
    TF_InitStatic(&tf, TF_SLAVE);  // or TF_SLAVE if device
    tf_uart_register_all_listeners();
    // 启动 UART DMA 接收
    HAL_UART_Receive_DMA(&huart1, uart_dma_buf, UART_DMA_BUF_SIZE);
    last_dma_pos = 0;
}

// === 每次轮询调用，处理新接收的数据 ===
void tf_uart_poll(void)
{
    uint16_t cur_pos = UART_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    uint32_t size    = 0;

    if (cur_pos != last_dma_pos) {
        if (cur_pos > last_dma_pos) {
            size = cur_pos - last_dma_pos;
            memcpy(temp_buf, &uart_dma_buf[last_dma_pos], size);
        } else {
            size = UART_DMA_BUF_SIZE - last_dma_pos;
            memcpy(temp_buf, &uart_dma_buf[last_dma_pos], size);
            memcpy(temp_buf + size, uart_dma_buf, cur_pos);
            size += cur_pos;
        }

        // 打印
        // printf("received:\n");
        // for (uint32_t i = 0; i < size; i++) { printf("0x%.2X\t", temp_buf[i]); }
        // printf("\r\n");

        TF_Accept(&tf, temp_buf, size);
        last_dma_pos = cur_pos;
    }
}

// helper func for testing
void dumpFrame(const uint8_t *buff, size_t len)
{
    size_t i;
    for (i = 0; i < len; i++) {
        printf("%3u %02X ", buff[i], buff[i]);
        if (buff[i] >= 0x20 && buff[i] < 127) {
            printf(" %c", buff[i]);
        } else {
            printf(" .");
        }
        printf("\n");
    }
    printf("--- end of frame ---\n\n");
}

void dumpFrameInfo(TF_Msg *msg)
{
    printf(
        "\033[33mFrame info\n"
        "  type: %02Xh\n"
        "  data: \"%.*s\"\n"
        "   len: %u\n"
        "    id: %Xh\033[0m\n\n",
        msg->type,
        msg->len,
        msg->data,
        msg->len,
        msg->frame_id);
}

// === 发送封装函数 ===
void tf_uart_send(uint8_t type, const uint8_t *data, uint32_t len)
{
    TF_Msg msg;
    msg.type = type;
    msg.data = data;
    msg.len  = len;
    TF_Send(&tf, &msg);
    // dumpFrameInfo(&msg);
    // dumpFrame(tf.sendbuf,10);
}

void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)buff, len, HAL_MAX_DELAY);
}

// ========================= Listeners ==========================

static TF_Result on_ctrl(TinyFrame *tf, TF_Msg *msg)
{
    printf("on_control\n");
    if (msg->len < 3 || msg->len >= 32) return TF_STAY;  // 最小长度限制 + 防止缓冲区溢出

    char buf[32];
    memcpy(buf, msg->data, msg->len);
    buf[msg->len] = '\0';  // 添加 null 结尾

    int index, pwm, time;
    if (sscanf(buf, "%d,%d,%d", &index, &pwm, &time) == 3) { servo_move_pwm(index, pwm, time); }

    return TF_STAY;
}

static TF_Result on_sync(TinyFrame *tf, TF_Msg *msg)
{
    printf("on_sync\n");

    char payload[128];
    int  len = snprintf(payload,
                       sizeof(payload),
                       "%d,%d,%d,%d,%d,%d",
                       servos[0].pwm,
                       servos[1].pwm,
                       servos[2].pwm,
                       servos[3].pwm,
                       servos[4].pwm,
                       servos[5].pwm);

    tf_uart_send(0x02, (const uint8_t *)payload, len);  // Reply type = 0x02
    return TF_STAY;
}

static TF_Result on_mode(TinyFrame *tf, TF_Msg *msg) { return TF_STAY; }

static TF_Result on_len(TinyFrame *tf, TF_Msg *msg)
{
    printf("on_len\n");
    char buf[32];
    memcpy(buf, msg->data, msg->len);
    buf[msg->len] = '\0';  // 添加 null 结尾

    int speed, deep;
    if (sscanf(buf, "%d,%d", &speed, &deep) == 2) { action_dymn_set(speed, deep); }
    return TF_STAY;
}

void tf_uart_register_all_listeners(void)
{
    TF_AddTypeListener(&tf, 0x01, on_ctrl);
    TF_AddTypeListener(&tf, 0x02, on_sync);
    TF_AddTypeListener(&tf, 0x03, on_mode);
    TF_AddTypeListener(&tf, 0x04, on_len);
}
