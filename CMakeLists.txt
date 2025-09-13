cmake_minimum_required(VERSION 3.22)
cmake_policy(SET CMP0077 NEW)

project(dof-firmware C CXX ASM)

set(IS_ROOT ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 20 CACHE STRING
        "The C++ standard whose features are requested to build this target")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Build type")
endif ()

message("Build type: ${CMAKE_BUILD_TYPE}")

if (CMAKE_BUILD_TYPE MATCHES "Release")
    set(LOG_VERBOSITY "INFO")
else ()
    set(LOG_VERBOSITY "DEBUG")
endif ()


set(CMAKE_SCRIPTS_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(${CMAKE_SCRIPTS_PATH}/make.cmake)

definePaths(${PROJECT_NAME})
defineProjectIncludes(${CMAKE_CURRENT_BINARY_DIR})


# Common STM32 definitions
add_compile_definitions(
        USE_HAL_DRIVER
        STM32F411xE
        STM32_THREAD_SAFE_STRATEGY=2
)

# Add debug definition for debug builds
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(DEBUG)
endif ()

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
        Core/Src/stm32f4xx_hal_msp.c
        Core/Src/stm32f4xx_it.c
        Core/Src/sysmem.c
)

# Set include directories for HAL library
target_include_directories(stm32_hal PUBLIC
        Core/Inc
        Drivers/STM32F4xx_HAL_Driver/Inc
        Drivers/CMSIS/Device/ST/STM32F4xx/Include
        Drivers/CMSIS/Include
        Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
)

add_subdirectory(src)

# Enable compile commands for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)
