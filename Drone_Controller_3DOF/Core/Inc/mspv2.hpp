/*
 * mspv2.hpp
 *
 *  Created on: Apr 12, 2023
 *      Author: Kerim
 */

#include "stm32f1xx_hal.h"   //** Change this according to your STM32 series **//
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#ifndef INC_MSPV2_HPP_
#define INC_MSPV2_HPP_


#define RNG_MSG_L 0x01
#define RNG_MSG_U 0x1f

#define OF_MSG_L 0x02
#define OF_MSG_U 0x1f



#define START_MSG 0x24


struct OF_Msg {
  uint8_t quality;
  int32_t motion_x;
  int32_t motion_y;
};

class MatekOF {
	public:
		OF_Msg of_msg_str;
		int32_t distance;
		UART_HandleTypeDef huart_of;

	private:
		uint16_t msg_type;
		uint8_t quality;
		uint8_t inChar, count;
		uint8_t matek_msg[30];
		char ck_in;
		uint16_t rng_msg =  RNG_MSG_U << 8 | RNG_MSG_L;
		uint16_t flow_msg  =  OF_MSG_U << 8 | OF_MSG_L;

	public:
		MatekOF();
		void MatekRead(void);
		void begin(UART_HandleTypeDef huart);
		~MatekOF();

	private:
		int32_t MatekDecodeRange(uint8_t* msg);
		OF_Msg MatekDecodeOF(uint8_t* msg);
};

#endif /* INC_MSPV2_HPP_ */
