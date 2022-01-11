################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../modular_robots/modular_ws/src/controller/include/src/Controller.cpp \
../modular_robots/modular_ws/src/controller/include/src/Kalman.cpp \
../modular_robots/modular_ws/src/controller/include/src/PID.cpp 

OBJS += \
./modular_robots/modular_ws/src/controller/include/src/Controller.o \
./modular_robots/modular_ws/src/controller/include/src/Kalman.o \
./modular_robots/modular_ws/src/controller/include/src/PID.o 

CPP_DEPS += \
./modular_robots/modular_ws/src/controller/include/src/Controller.d \
./modular_robots/modular_ws/src/controller/include/src/Kalman.d \
./modular_robots/modular_ws/src/controller/include/src/PID.d 


# Each subdirectory must supply rules for building sources it contributes
modular_robots/modular_ws/src/controller/include/src/Controller.o: ../modular_robots/modular_ws/src/controller/include/src/Controller.cpp modular_robots/modular_ws/src/controller/include/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Kerim/STM32CubeIDE/STM32Quadrotor/Drone_Controller_3DOF/modular_robots/modular_ws/src/controller/include/controller" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"modular_robots/modular_ws/src/controller/include/src/Controller.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
modular_robots/modular_ws/src/controller/include/src/Kalman.o: ../modular_robots/modular_ws/src/controller/include/src/Kalman.cpp modular_robots/modular_ws/src/controller/include/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Kerim/STM32CubeIDE/STM32Quadrotor/Drone_Controller_3DOF/modular_robots/modular_ws/src/controller/include/controller" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"modular_robots/modular_ws/src/controller/include/src/Kalman.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
modular_robots/modular_ws/src/controller/include/src/PID.o: ../modular_robots/modular_ws/src/controller/include/src/PID.cpp modular_robots/modular_ws/src/controller/include/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m3 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Kerim/STM32CubeIDE/STM32Quadrotor/Drone_Controller_3DOF/modular_robots/modular_ws/src/controller/include/controller" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -MMD -MP -MF"modular_robots/modular_ws/src/controller/include/src/PID.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

