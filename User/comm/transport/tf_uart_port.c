#include "tf_uart_port.h"

#include <stdio.h>
#include <string.h>

#include "../drivers/uart_driver.h"
#include "tinyframe/TinyFrame.h"
#include "tinyframe/utils.h"

// ==================== Static State ====================
static TinyFrame tf_instance;  // TinyFrame instance

static tf_uart_frame_callback_t user_callback = NULL;  // User callback
static bool tf_uart_inited = false;

#ifndef TF_UART_PORT_LOG_ENABLE
#define TF_UART_PORT_LOG_ENABLE 1
#endif

#if TF_UART_PORT_LOG_ENABLE
#define TF_UART_LOG(fmt, ...) printf("[tf_uart] " fmt "\n", ##__VA_ARGS__)
#else
#define TF_UART_LOG(fmt, ...) ((void)0)
#endif

// ==================== TinyFrame callbacks ====================

/**
 * @brief TinyFrame write implementation (send data to UART)
 */
void TF_WriteImpl(TinyFrame* tf, const uint8_t* buff, uint32_t len)
{
    (void)tf;
    if (buff == NULL || len == 0) {
        TF_UART_LOG("UART send skipped (empty buffer)");
        return;
    }
    // Use UART async send
    if (uart_driver_send_async(buff, len) != 0) {
        TF_UART_LOG("UART send failed");
    } else {
        TF_UART_LOG("UART send success");
        dumpFrame(buff, len);
    }
}

/**
 * @brief TinyFrame generic listener
 */
static TF_Result tf_frame_listener(TinyFrame* tf, TF_Msg* msg)
{
    TF_UART_LOG("UART received:");
    dumpFrameInfo(msg);
    TF_Send(tf, msg);
    // Call user callback (frame type and payload)
    if (user_callback != NULL) {
        user_callback(msg->type, msg->data, msg->len);
    }
    // Keep listener
    return TF_STAY;
}

// ==================== UART RX callback ====================

/**
 * @brief UART receive callback
 */
static void uart_rx_callback(const uint8_t* data, uint32_t len)
{
    if (data == NULL || len == 0) {
        return;
    }
    // Pass raw bytes to TinyFrame
    TF_Accept(&tf_instance, data, len);
    // printf("[tf_uart] uart received: %.*s\n", len, data);
}

// ==================== Public API ====================

bool tf_uart_port_init(tf_uart_frame_callback_t callback)
{
    if (tf_uart_inited) {
        TF_UART_LOG("already initialized");
        return true;
    }

    TF_UART_LOG("init tf_uart_port");

    // Save callback
    user_callback = callback;

    // 1. Init UART driver
    if (uart_driver_init(uart_rx_callback) != 0) {
        TF_UART_LOG("UART driver init failed");
        return false;
    }

    // 2. Init TinyFrame (slave)
    TF_InitStatic(&tf_instance, TF_SLAVE);

    // 3. Register TinyFrame generic listener
    TF_AddGenericListener(&tf_instance, tf_frame_listener);

    tf_uart_inited = true;

    TF_UART_LOG("init tf_uart_port success");
    TF_UART_LOG("user callback: %p", (void*)callback);

    return true;
}

bool tf_uart_port_send_frame(uint8_t frame_type, const uint8_t* data, uint16_t len)
{
    // Check init
    if (!tf_uart_inited) {
        TF_UART_LOG("error: have not init port");
        return false;
    }
    if (data == NULL || len == 0) {
        TF_UART_LOG("error: empty frame");
        return false;
    }

    // Build TinyFrame message
    TF_Msg msg;
    msg.type = frame_type;  // Frame type set by caller
    msg.data = data;        // Payload
    msg.len  = len;         // Length

    // Send frame
    bool success = TF_Send(&tf_instance, &msg);

    if (success) {
        TF_UART_LOG("send frame: type=0x%02X, len=%u", frame_type, len);
    } else {
        TF_UART_LOG("send failed");
    }

    return success;
}

void tf_uart_port_poll(void)
{
    // Poll UART driver (process RX data)
    uart_driver_poll();
}

void tf_uart_port_tick_1ms(void)
{
    // Handle TinyFrame timeouts
    TF_Tick(&tf_instance);
}

bool tf_uart_port_is_tx_done(void)
{
    return uart_driver_is_tx_done();
}

void* tf_uart_port_get_instance(void)
{
    return (void*)&tf_instance;
}
