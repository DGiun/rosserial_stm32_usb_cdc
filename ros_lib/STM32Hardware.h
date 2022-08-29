/*
 * STM32Hardware.h
 *
 *  Created on: Fed 10, 2022
 *      Author: DGiun
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#include "BufferedSerial.hpp"

#ifdef USB

// Change "CDC_Transmit_FS" and "CDC_Receive_FS" Function
// in '../USB_DEVICE/App/usbd_cdc_if.c'

#include "usbd_cdc_if.h"

extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint32_t rx_head;

class STM32Hardware {
 private:
	uint32_t rx_tail;

 public:
  STM32Hardware(){}

  // Any initialization code necessary to use the serial port:
  void init() {
	  rx_head = rx_tail = 0u;
  }

  // Read a byte from the serial port. -1 = failure:
  int read() {
	  // Quit if no new character:
	  if (rx_head == rx_tail){
		  rx_head = rx_tail = 0u;
		  return -1;
	  }

	  // Get next char in buffer:
	  return static_cast<int>(UserRxBufferFS[rx_tail++]);
  }

  // Write data to the connection to ROS:
  void write(uint8_t* data, int length) {
	  CDC_Transmit_FS((uint8_t *)data, length);
  }

  // Returns milliseconds since start of program:
  unsigned long time() { return HAL_GetTick(); };
};
#endif /* USB */

#ifdef UART

// Create Serial Buffer with UART2:
extern BufferedSerial buff_serial;

class STM32Hardware {
 public:
  STM32Hardware() : serial(&buff_serial) {}

  // Any initialization code necessary to use the serial port:
  void init() { serial->init(); }

  // Read a byte from the serial port. -1 = failure:
  int read() { return serial->read(); }

  // Write data to the connection to ROS:
  void write(uint8_t* data, int length) { serial->write(data, length); }

  // Returns milliseconds since start of program:
  unsigned long time() { return HAL_GetTick(); };

 protected:
  BufferedSerial* serial;
};

#endif /* UART */

#endif

