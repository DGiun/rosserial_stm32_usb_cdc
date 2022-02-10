/*
 * BufferedSerial.h
 *
 *  Created on: Fed 10, 2022
 *      Author: DGiun
 */

#ifndef BUFFEREDSERIAL_H_
#define BUFFEREDSERIAL_H_

#include <string.h>

#define STM32F4xx  // Change for your device

#ifdef STM32F3xx
#include "stm32f3xx_hal.h"
#include "stm32f3xx_hal_uart.h"
#endif /* STM32F3xx */
#ifdef STM32F4xx
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#endif /* STM32F4xx */
#ifdef STM32F7xx
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#endif /* STM32F7xx */

#define RX_BUF_SIZE 512
#define TX_BUF_SIZE 512

class BufferedSerial {
 private:
  // UART:
  UART_HandleTypeDef &huart;

  // Buffers:
  static constexpr uint16_t rx_buf_mask = RX_BUF_SIZE - 1;
  static constexpr uint16_t tx_buf_mask = TX_BUF_SIZE - 1;
  uint8_t rx_buf[RX_BUF_SIZE];
  uint8_t tx_buf[TX_BUF_SIZE];

  // Indexes:
  uint16_t rx_tail = 0;
  uint16_t tx_head = 0;
  uint16_t tx_tail = 0;
  uint16_t tx_end = TX_BUF_SIZE;

 public:
  BufferedSerial(UART_HandleTypeDef &huart);
  BufferedSerial();
  UART_HandleTypeDef *const get_handle(void);

  void init(void);
  int read(void);
  void write(const uint8_t *const c, const int length);
  void flush_tx_buffer();

  void tx_cplt_callback(void);
  void reset_rx_buffer(void);
};

#endif /* BUFFEREDSERIAL_H_ */
