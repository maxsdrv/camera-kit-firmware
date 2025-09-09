set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Some default GCC settings
# arm-none-eabi- must be part of path environment
set(TOOLCHAIN_PREFIX                /Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-)

set(CMAKE_C_COMPILER                ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER              ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER              ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_LINKER                    ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_OBJCOPY                   ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_SIZE                      ${TOOLCHAIN_PREFIX}size)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags - simplified to avoid duplication
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard")

# Set basic compiler flags
set(CMAKE_C_FLAGS_INIT "${TARGET_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS_INIT "${TARGET_FLAGS} -x assembler-with-cpp")
set(CMAKE_CXX_FLAGS_INIT "${TARGET_FLAGS} -Wall -Wextra -Wpedantic -fdata-sections -ffunction-sections -fno-rtti -fno-exceptions -fno-threadsafe-statics")

# Debug/Release specific flags
set(CMAKE_C_FLAGS_DEBUG_INIT "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE_INIT "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG_INIT "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Os -g0")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS_INIT "${TARGET_FLAGS} -T\"${CMAKE_CURRENT_LIST_DIR}/../STM32F411XX_FLASH.ld\" --specs=nano.specs -Wl,--gc-sections -Wl,--print-memory-usage")
