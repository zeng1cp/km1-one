#ifndef TF_UART_PORT_H
#define TF_UART_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 帧接收回调函数类型
 * @param frame_type 帧类型（由上层协议定义）
 * @param data 帧数据指针
 * @param len 帧数据长度
 */
typedef void (*tf_uart_frame_callback_t)(uint8_t frame_type, const uint8_t* data, uint16_t len);

/**
 * @brief 初始化通信端口
 * @param callback 帧接收回调函数
 * @return true:成功, false:失败
 */
bool tf_uart_port_init(tf_uart_frame_callback_t callback);

/**
 * @brief 发送一帧数据
 * @param frame_type 帧类型（由上层协议定义）
 * @param data 数据指针
 * @param len 数据长度
 * @return true:成功, false:失败
 */
bool tf_uart_port_send_frame(uint8_t frame_type, const uint8_t* data, uint16_t len);

/**
 * @brief 轮询处理（在主循环中调用）
 */
void tf_uart_port_poll(void);

/**
 * @brief tick(每1ms调用一次)
 */
void tf_uart_port_tick_1ms(void);

/**
 * @brief 检查发送是否完成
 * @return true:完成, false:进行中
 */
bool tf_uart_port_is_tx_done(void);

/**
 * @brief 获取TinyFrame实例指针
 *
 * @return TinyFrame实例指针，可用于注册监听器等操作
 */
void* tf_uart_port_get_instance(void);

#ifdef __cplusplus
}
#endif

#endif  // TF_UART_PORT_H
