#include "gy-us42v2.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
static int u_counter;
char read;
char write = 1;
char error;

void request_range(void) {



		//char error = 1;

		unsigned char command = 0x51;
		char error = HAL_I2C_Master_Transmit(&hi2c1, 224, &command, 1, I2C_TIMEOUT);
		//HAL_Delay(100);
		write = 0;

}

			//HAL_I2C_Master_Transmit(&hi2c1, 224, &command, 1, I2C_TIMEOUT);
int getRange (void)
				{
		//if (!error) {
			read = 0;
			write = 1;
			unsigned char range[2];
			HAL_I2C_Master_Receive(&hi2c1, 225, range, 2, I2C_TIMEOUT);
			return (range[0] << BYTE_SHIFT) | range[1];
		//}



	//return -1;

}

void set_ucounter(void) {
	u_counter++;
	if(u_counter > 40) {
		u_counter = 0;
	}
}

int get_ucounter(void) {
	return u_counter;
}


