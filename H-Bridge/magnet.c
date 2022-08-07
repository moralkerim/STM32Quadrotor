/*
 * magnet.c
 *
 *  Created on: Aug 7, 2022
 *      Author: Bader
 */


void SwitchMag(int ch_state);

void SwitchMag(int ch_state) {
	
	char state;
	
	// determine which state is the switch on
	if		(ch_state > 750  && ch_state < 1250) state = 0;
	else if (ch_state > 1250 && ch_state < 1750) state = 1;
	else if (ch_state > 1750 && ch_state < 2250) state = 2;
	else state = -1;

	// change magnet state based on state of switch
	// state = 0 -> off
	// state = 1 -> attach
	// state = 2 -> separate

	switch(state) {
	case 0:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		break;
	default:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
		break;
	}
}
