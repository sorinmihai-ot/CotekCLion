cmake_minimum_required(VERSION 3.22)

# Toolchain/ABI is provided by your cmake/arm-none-eabi.cmake profile
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(CotekCLion C ASM)
enable_language(C ASM)



set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)

# --- QP/Spy toggle (optional) ---
option(USE_QSPY "Enable QP/Spy tracing (QS)" OFF)

# QPC root (relative to project root)
set(QPC_DIR "${CMAKE_SOURCE_DIR}/qpc")

# --- Common compile flags ---
add_compile_options(
        -mcpu=cortex-m3
        -mthumb
        -ffunction-sections -fdata-sections
        -fno-common
        -fmessage-length=0
        -Wall -Wextra -Wundef -Werror=return-type
)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_compile_options(-Ofast)
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    add_compile_options(-Ofast -g)
elseif (CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
    add_compile_options(-Os)
else()
    add_compile_options(-Og -g)
endif()

# allow .s with preprocessor
string(APPEND CMAKE_ASM_FLAGS " -x assembler-with-cpp")

# --- Startup autodetect ---
file(GLOB STARTUP_CANDIDATES
        "${CMAKE_SOURCE_DIR}/Core/Startup/startup_stm32f*.s"
        "${CMAKE_SOURCE_DIR}/Startup/startup_stm32f*.s"
        "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Source/Templates/gcc/startup_stm32f*.s"
)
list(LENGTH STARTUP_CANDIDATES _n)
if (_n EQUAL 0)
    message(FATAL_ERROR "startup_stm32f*.s not found")
endif()
list(GET STARTUP_CANDIDATES 0 STARTUP_FILE)
set_source_files_properties(${STARTUP_FILE} PROPERTIES LANGUAGE ASM)
message(STATUS "Using startup file: ${STARTUP_FILE}")

# --- Linker script ---
set(LINKER_SCRIPT "${CMAKE_SOURCE_DIR}/STM32F103RBTX_FLASH.ld")
if (NOT EXISTS "${LINKER_SCRIPT}")
    file(GLOB LD_ALTS "${CMAKE_SOURCE_DIR}/*.ld")
    if (LD_ALTS)
        list(GET LD_ALTS 0 LINKER_SCRIPT)
        message(WARNING "Default LD not found; using detected: ${LINKER_SCRIPT}")
    else()
        message(FATAL_ERROR "No linker script (*.ld) found.")
    endif()
endif()
message(STATUS "Using linker script: ${LINKER_SCRIPT}")

# --- Source files ---
file(GLOB_RECURSE APP_SOURCES
        "${CMAKE_SOURCE_DIR}/Core/Src/*.c"
        "${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Src/*.c"
        "${QPC_DIR}/src/qep/*.c"
        "${QPC_DIR}/src/qf/*.c"
        "${QPC_DIR}/src/qv/*.c"
)
list(FILTER APP_SOURCES EXCLUDE REGEX "_template\\.c$")

# add ARM-CM QV port glue if present
if (EXISTS "${QPC_DIR}/ports/arm-cm/qv/gnu/qv_port.c")
    list(APPEND APP_SOURCES "${QPC_DIR}/ports/arm-cm/qv/gnu/qv_port.c")
endif()

add_executable(CotekCLion.elf ${APP_SOURCES} ${STARTUP_FILE}
        Core/Inc/qpc_cfg.h
        Core/Inc/batt_classify.h
        Core/Src/batt_classify.c
        Core/Inc/can_proto.h
        Core/Src/bsut.c
        Core/Src/bms_fault_decode.c
)

# --- Include paths ---
target_include_directories(CotekCLion.elf PRIVATE
        "${CMAKE_SOURCE_DIR}/Core/Inc"
        "${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc"
        "${CMAKE_SOURCE_DIR}/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy"
        "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F1xx/Include"
        "${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include"
        "${QPC_DIR}/include"
        "${QPC_DIR}/ports/arm-cm/qv/gnu"
)

# --- Preprocessor defs ---
target_compile_definitions(CotekCLion.elf PRIVATE
        USE_HAL_DRIVER
        STM32F103xB
        QF_BASEPRI=0x50  # 0x50 >> (8-4) = 5
        #ENABLE_BMS_SIM
        $<$<CONFIG:Debug>:DEBUG>
)

# --- Link options ---
target_link_options(CotekCLion.elf PRIVATE
        "-T${LINKER_SCRIPT}"
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        "-Wl,-Map=${PROJECT_BINARY_DIR}/CotekCLion.map"
        -mcpu=cortex-m3 -mthumb
        -specs=nano.specs -specs=nosys.specs
)

set_target_properties(CotekCLion.elf PROPERTIES
        LINK_FLAGS "-u _printf_float -u _scanf_float"
)

target_link_libraries(CotekCLion.elf m)

# --- HEX/BIN artifacts ---
set(HEX_FILE "${PROJECT_BINARY_DIR}/CotekCLion.hex")
set(BIN_FILE "${PROJECT_BINARY_DIR}/CotekCLion.bin")
add_custom_command(TARGET CotekCLion.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex  $<TARGET_FILE:CotekCLion.elf> "${HEX_FILE}"
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:CotekCLion.elf> "${BIN_FILE}"
        COMMAND ${CMAKE_SIZE}    $<TARGET_FILE:CotekCLion.elf>
        COMMENT "Built ${HEX_FILE} / ${BIN_FILE}"
)

# --- Flash helper target (CubeProgrammer CLI) ---
add_custom_target(flash
        COMMAND "C:/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"
        -c port=SWD mode=UR
        -w "${HEX_FILE}"
        -v -rst
        DEPENDS CotekCLion.elf
        COMMENT "Flashing via STM32CubeProgrammer"
)
