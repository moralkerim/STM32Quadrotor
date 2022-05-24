#include "gy-us42v2.h"
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
int getRange (void)
{

			//char error = 1;
			unsigned char command = 0x51;
			char error = HAL_I2C_Master_Transmit(&hi2c1, 224, &command, 1, I2C_TIMEOUT);
			//HAL_I2C_Master_Transmit(&hi2c1, 224, &command, 1, I2C_TIMEOUT);

				if (!error) {
					unsigned char range[2];
					HAL_I2C_Master_Receive(&hi2c1, 225, range, 2, I2C_TIMEOUT);
					return (range[0] << BYTE_SHIFT) | range[1];
				}



	return -1;

}
