cmake_minimum_required(VERSION 3.22)
cmake_policy(SET CMP0077 NEW)

project(dof-firmware C CXX ASM)

set(IS_ROOT ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 20 CACHE STRING
    "The C++ standard whose features are requested to build this target")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type")
endif()

message("Build type: ${CMAKE_BUILD_TYPE}")

if (CMAKE_BUILD_TYPE MATCHES "Release")
    set(LOG_VERBOSITY "INFO")
else()
    set(LOG_VERBOSITY "DEBUG")
endif()

if (NOT DEFINED MODULES_GLOBAL)
    set(MODULES_GLOBAL ${CMAKE_CURRENT_SOURCE_DIR}/modules)
endif()

if (MODULES_USE_GLOBAL)
    set(MODULES ${MODULES_GLOBAL})
else()
    set(MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules)
endif()

set(CMAKE_SCRIPTS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(${CMAKE_SCRIPTS_PATH}/make.cmake)

definePaths(${PROJECT_NAME})
defineProjectIncludes(${CMAKE_CURRENT_BINARY_DIR})

# DOF Firmware Configuration
set(DOF_CONFIGURATION "full" CACHE STRING "DOF firmware configuration")
if(DOF_CONFIGURATION MATCHES "full")
    set(WITH_COMMUNICATION ON)
    set(WITH_TELEMETRY ON)
    set(WITH_GIMBAL_CONTROL ON)
elseif(DOF_CONFIGURATION MATCHES "gimbal-only")
    set(WITH_GIMBAL_CONTROL ON)
elseif(DOF_CONFIGURATION MATCHES "minimal")
    set(WITH_COMMUNICATION ON)
    set(WITH_TELEMETRY ON)
endif()

# Enable coroutines support (if available)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fcoroutines")
endif()

# Define STM32 compiler flags
set(STM32_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -fdata-sections
    -ffunction-sections
    -Wall
    -Wextra
    -Wpedantic
    -fno-exceptions
    -fno-rtti
)

# Define additional STM32 linker flags
set(STM32_ADDITIONAL_LINKER_FLAGS
    "-Wl,--defsym=malloc_getpagesize_P=0x80"
)

# Common STM32 definitions
add_compile_definitions(
    USE_HAL_DRIVER
    STM32F411xE
    STM32_THREAD_SAFE_STRATEGY=2
)

# Add debug definition for debug builds
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(DEBUG)
endif()

# Create HAL library
add_library(stm32_hal STATIC
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# Set include directories for HAL library
target_include_directories(stm32_hal PUBLIC
    src/board/Inc
    Drivers/STM32F4xx_HAL_Driver/Inc
    Drivers/CMSIS/Device/ST/STM32F4xx/Include
    Drivers/CMSIS/Include
    Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
)

add_subdirectory(src)

if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Enable compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
