cmake_minimum_required (VERSION 3.14)

set (CMAKE_SYSTEM_NAME Generic)
set (CMAKE_SYSTEM_PROCESSOR cortex-m3)
set (CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set (CMAKE_C_COMPILER arm-none-eabi-gcc)
set (CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set (CMAKE_C_STANDARD 11)

project (spitest C ASM)


find_program (OBJCOPY arm-none-eabi-objcopy)
find_program (SIZE arm-none-eabi-size)
find_program (ST_FLASH st-flash)

set (STM32CUBE "" CACHE STRING "STM32Cube toolkit")

add_executable (spitest 
  src/main.c
  src/stm32f1xx_it.c
  src/system_stm32f1xx.c
)

target_compile_definitions (spitest PUBLIC
  STM32
  STM32F1
  STM32F103xB
  USE_FULL_LL_DRIVER
  USE_FULL_ASSERT
)

target_include_directories (spitest SYSTEM PUBLIC
  ${STM32CUBE}/Drivers/CMSIS/Include
  ${STM32CUBE}/Drivers/CMSIS/Device/ST/STM32F1xx/Include
  ${STM32CUBE}/Drivers/STM32F1xx_HAL_Driver/Inc
)

target_include_directories (spitest PUBLIC
  ${PROJECT_SOURCE_DIR}/src
)

target_sources (spitest PUBLIC
  ${STM32CUBE}/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f103xb.s
  ${STM32CUBE}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c
  ${STM32CUBE}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c
  ${STM32CUBE}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c
  ${STM32CUBE}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_exti.c
  ${STM32CUBE}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_dma.c
  ${STM32CUBE}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_spi.c
  ${STM32CUBE}/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_pwr.c
)

target_link_options (spitest PUBLIC
  -Wl,--gc-sections
  -specs=nosys.specs
  -mcpu=cortex-m3
  -mlittle-endian
  -mfloat-abi=soft
  -T STM32F103XB_FLASH.ld -L ${PROJECT_SOURCE_DIR}/ldscripts
)

target_compile_options (spitest PUBLIC
  -Wno-unused-parameter
  -Wall
  -Wextra
  -mthumb
  -mcpu=cortex-m3
  -mlittle-endian
  -mfloat-abi=soft
  -ffunction-sections 
  -fdata-sections
)

add_custom_target (spitest.bin ALL
  COMMAND ${SIZE} spitest
  COMMAND ${OBJCOPY} -S -O binary spitest spitest.bin
  DEPENDS spitest
)

add_custom_target (flash
  COMMAND ${ST_FLASH} write spitest.bin 0x08000000
  DEPENDS spitest.bin
)
