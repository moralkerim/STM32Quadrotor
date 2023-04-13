#include "mspv2.hpp"
/*
 * mspv2.cpp
 *
 *  Created on: Apr 12, 2023
 *      Author: Kerim
 */

MatekOF::MatekOF() {};

void MatekOF::begin(UART_HandleTypeDef huart) {
	memcpy(&huart_of, &huart, sizeof(huart));
}

void MatekOF::MatekRead() {
		HAL_UART_Receive_DMA (&huart_of, (uint8_t*)&inChar, 1);
		count++;
		if(inChar == START_MSG) {
				count = 0;
				matek_msg[count] = START_MSG;
				if((matek_msg[12] | matek_msg[11] | matek_msg[10] | matek_msg[9]) != 0) {
					msg_type = matek_msg[5] << 8 | matek_msg[4];
					quality = matek_msg[8];
					if(msg_type == rng_msg) {
						distance = MatekDecodeRange(matek_msg);
						//payload_size = matek_msg[7] << 8 | matek_msg[6];

					}

					else if(msg_type == flow_msg) {
						of_msg_str = MatekDecodeOF(matek_msg);
					}
				}
		}
		else {
			matek_msg[count] = inChar;
		}


}


int32_t MatekOF::MatekDecodeRange(uint8_t* msg) {
  int32_t distance = msg[9] | msg[10] << 8 | msg[11] << 16 | msg[12] << 24 ;
  return distance;
}

OF_Msg MatekOF::MatekDecodeOF(uint8_t* msg) {
  OF_Msg of_msg;
  of_msg.quality = msg[8];
  of_msg.motion_x = msg[9] | msg[10] << 8 | msg[11] << 16 | msg[12] << 24;
  of_msg.motion_y = msg[13] | msg[14] << 8 | msg[15] << 16 | msg[16] << 24 ;
  return of_msg;
}

MatekOF::~MatekOF() {};
