#include "stdint.h"

#define BYTE_SHIFT					(8U)
#define I2C_TIMEOUT 100 //Define a timeout of 100 ms -- do not wait for clock stretching longer than this time

int getRange (void);
