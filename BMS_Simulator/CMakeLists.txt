cmake_minimum_required(VERSION 3.28)

# ------------------------------------------------------------
# Cross-compiling for STM32L452RE (Cortex-M4F, no FPU hardfloat)
# ------------------------------------------------------------
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(BatterySIM_L452 C CXX ASM)

set(CMAKE_C_STANDARD 11)

# ------------------------------------------------------------
# Toolchain programs
# ------------------------------------------------------------
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE    arm-none-eabi-size)

# ------------------------------------------------------------
# CPU / ABI flags as a LIST (very important!)
# ------------------------------------------------------------
set(MCU_FLAGS
        -mcpu=cortex-m4
        -mthumb
        -mthumb-interwork
        -mfloat-abi=soft
)

# Common warnings / section flags
set(COMMON_FLAGS
        -ffunction-sections
        -fdata-sections
        -fno-common
        -fmessage-length=0
)

# Optimisation per build type
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    set(OPT_FLAGS -Ofast)
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(OPT_FLAGS -Ofast -g3)
elseif (CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    set(OPT_FLAGS -Os)
else()
    # Debug default
    set(OPT_FLAGS -Og -g3)
endif()

message(STATUS "Using build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Opt flags: ${OPT_FLAGS}")

# Tell CMake what flags to use when compiling C and ASM
add_compile_options(
        ${MCU_FLAGS}
        ${COMMON_FLAGS}
        ${OPT_FLAGS}
        -std=gnu11
)

# The startup assembly file needs preprocessing
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -x assembler-with-cpp")

# ------------------------------------------------------------
# Preprocessor defines
# ------------------------------------------------------------
add_definitions(
        -DDEBUG
        -DUSE_HAL_DRIVER
        -DSTM32L452xx
)

# ------------------------------------------------------------
# Include directories
# ------------------------------------------------------------
include_directories(
        ${CMAKE_SOURCE_DIR}/Core/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32L4xx_HAL_Driver/Inc/Legacy
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32L4xx/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)

# ------------------------------------------------------------
# Source files
# (Update the startup .s filename to match what CubeMX generated)
# ------------------------------------------------------------
set(CORE_SRCS
        Core/Src/main.c
        Core/Src/gpio.c
        Core/Src/i2c.c
        Core/Src/usart.c
        Core/Src/can.c
        Core/Src/stm32l4xx_it.c
        Core/Src/stm32l4xx_hal_msp.c
        Core/Src/system_stm32l4xx.c

        # our custom app code
        Core/Src/lcd_2004.c
        Core/Src/buttons.c
        Core/Src/can_tx.c
        Core/Src/app_state.c

        # syscalls / heap hooks (printf etc.)
        Core/Src/syscalls.c
        Core/Src/sysmem.c
)

set(HAL_SRCS
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_cortex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_rcc_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_gpio.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_dma_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_pwr_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_i2c_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_uart_ex.c
        Drivers/STM32L4xx_HAL_Driver/Src/stm32l4xx_hal_can.c
)

# check your actual startup filename in Core/Startup
set(STARTUP_SRCS
        Core/Startup/startup_stm32l452retx.s
        Core/Inc/can_400HYP.h
        Core/Src/can_400HYP.c
        Core/Inc/can_400DZB.h
        Core/Src/can_400DZB.c
        Core/Inc/can_400ST.h
        Core/Src/can_400ST.c
        Core/Src/can_600.c
        Core/Inc/can_600.h
        Core/Src/can_500BMZ.c
        Core/Inc/can_500BMZ.h
        Core/Src/can_500HYP.c
        Core/Inc/can_500HYP.h
        Core/Src/can_sniffer.c
        Core/Src/can_sniffer.h
)

# Linker script from CubeMX (now in the project root)
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32L452RETX_FLASH.ld)

# ------------------------------------------------------------
# Create the ELF target
# ------------------------------------------------------------
add_executable(BatterySIM_L452.elf
        ${CORE_SRCS}
        ${HAL_SRCS}
        ${STARTUP_SRCS}
)

# ------------------------------------------------------------
# Linker options
# We must give the same MCU flags to the linker as separate list items.
# We also point the linker at the script using -T <filename>
# and add -L <dir> so it can find it.
# ------------------------------------------------------------
# Linker script and core link flags
target_link_options(BatterySIM_L452.elf PRIVATE
        ${MCU_FLAGS}
        -Wl,-gc-sections
        -Wl,--print-memory-usage
        -Wl,-Map=${PROJECT_BINARY_DIR}/BatterySIM_L452.map
        -L${CMAKE_SOURCE_DIR}
        -TSTM32L452RETX_FLASH.ld
)

# Standard baremetal libs (no -lnosys now)
target_link_libraries(BatterySIM_L452.elf
        -Wl,--start-group
        c m gcc
        -Wl,--end-group
)

# ------------------------------------------------------------
# Post-build: create HEX/BIN and show size
# ------------------------------------------------------------
add_custom_command(TARGET BatterySIM_L452.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex   BatterySIM_L452.elf BatterySIM_L452.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary BatterySIM_L452.elf BatterySIM_L452.bin
        COMMAND ${CMAKE_SIZE} BatterySIM_L452.elf
        COMMENT "Generating HEX, BIN and size info"
)

# ------------------------------------------------------------
# Flash helper target (optional)
# ------------------------------------------------------------
add_custom_target(flash
        COMMAND ${CMAKE_COMMAND} -E echo "Flashing BatterySIM_L452.hex to target via ST-Link"
        COMMAND "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"
        -c port=SWD mode=UR reset=HWrst
        -w BatterySIM_L452.hex
        -rst
        DEPENDS BatterySIM_L452.elf
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Flashing firmware"
)
