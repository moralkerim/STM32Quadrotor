#include "mspv2.hpp"
/*
 * mspv2.cpp
 *
 *  Created on: Apr 12, 2023
 *      Author: Kerim
 */

MatekOF::MatekOF() {};

void MatekOF::Pix2Meter() {
	float range_m = (float)distance/1000.0;
	vel_x = of_msg_str.motion_x * 2 * range_m * tan(alpha/2) / W;
	vel_y = of_msg_str.motion_y * 2 * range_m * tan(alpha/2) / W;
}

void MatekOF::begin(UART_HandleTypeDef huart) {
	memcpy(&huart_of, &huart, sizeof(huart));
}

void MatekOF::MatekRead2() {
	uint8_t start_index;
	uint8_t in_msg[18];
	for(int i=0; i<sizeof(matek_msg2); i++) {
		if(matek_msg2[i] == START_MSG) {
				start_index = i;
				break;
			}
		}
	msg_type = matek_msg2[(start_index+5)%36] << 8 | matek_msg2[(start_index+4)%36];
	if(msg_type == rng_msg) {
		flow_time_ = flow_time;
		flow_time = HAL_GetTick();
		flow_time_diff = flow_time-flow_time_;
		for(int i=start_index; i<start_index+14; i++) {
			in_msg[i - start_index] = matek_msg2[i%36];
		}
		int32_t meas_dist  = MatekDecodeRange(in_msg);
		if(CheckRange(meas_dist)) {
			distance = meas_dist;
		}
		//payload_size = matek_msg[7] << 8 | matek_msg[6];

	}

	else if(msg_type == flow_msg) {

		for(int i=start_index; i<start_index+18; i++) {
			in_msg[i - start_index] = matek_msg2[i%36];
		}
		OF_Msg of_msg_proto = MatekDecodeOF(in_msg);
		if(CheckMotion(of_msg_proto)) {
			of_msg_str = of_msg_proto;
			//Pix2Meter();
		}
	}
	HAL_UART_Receive_DMA(&huart_of, (uint8_t*)matek_msg2, 36);

}

void MatekOF::MatekRead() {
		count++;
		if(inChar == START_MSG) {
				count = 0;
				matek_msg[count] = START_MSG;
				if((matek_msg[12] | matek_msg[11] | matek_msg[10] | matek_msg[9]) != 0) {
					msg_type = matek_msg[5] << 8 | matek_msg[4];
					quality = matek_msg[8];
					if(msg_type == rng_msg) {
						flow_time_ = flow_time;
						flow_time = HAL_GetTick();
						flow_time_diff = flow_time-flow_time_;
						int32_t meas_dist  = MatekDecodeRange(matek_msg);
						if(CheckRange(meas_dist)) {
							distance = meas_dist;
						}
						//payload_size = matek_msg[7] << 8 | matek_msg[6];

					}

					else if(msg_type == flow_msg) {

						OF_Msg of_msg_proto = MatekDecodeOF(matek_msg);
						if(CheckMotion(of_msg_proto)) {
							of_msg_str = of_msg_proto;
							Pix2Meter();
						}
					}
				}
		}
		else {
			matek_msg[count] = inChar;
		}

		HAL_UART_Receive_DMA (&huart_of, (uint8_t*)&inChar, 1);
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

bool MatekOF::CheckRange(int32_t range) {
	bool range_ok;
	if(range <0 || range > 2000) {
		range_ok = false;
	}

	else {
		range_ok = true;
	}

	return range_ok;
}

bool MatekOF::CheckMotion(OF_Msg of_msg) {
	bool mot_ok;
	if(abs(of_msg.motion_x) > 500 || abs(of_msg.motion_y) > 500) {
		mot_ok = false;
	}

	else {
		mot_ok = true;
	}

	return mot_ok;
}

MatekOF::~MatekOF() {};
