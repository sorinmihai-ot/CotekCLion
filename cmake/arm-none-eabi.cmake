# cmake/arm-none-eabi.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Adjust paths if your install dir differs
set(TOOLROOT "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.3 rel1/bin")
set(CMAKE_C_COMPILER   "${TOOLROOT}/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${TOOLROOT}/arm-none-eabi-gcc.exe")
set(CMAKE_AR           "${TOOLROOT}/arm-none-eabi-ar.exe")
set(CMAKE_OBJCOPY      "${TOOLROOT}/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE         "${TOOLROOT}/arm-none-eabi-size.exe")
