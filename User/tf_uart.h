#ifndef __TF_UART_H__
#define __TF_UART_H__

#include "TinyFrame.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

void tf_uart_init(void);
void tf_uart_tick(void);  // optional: call this in systick or timer
void tf_uart_send(uint8_t type, const uint8_t *data, uint32_t len);

// To be called inside HAL_UART_RxCpltCallback
void tf_uart_poll(void);

// Application-specific listeners
void tf_uart_register_all_listeners(void);

#ifdef __cplusplus
}
#endif

#endif /*__TF_UART_H__*/

