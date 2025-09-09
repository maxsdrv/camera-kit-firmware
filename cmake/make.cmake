# STM32 Project Build System
# This file provides common build functions and configuration

cmake_minimum_required(VERSION 3.22)

# Set policies
if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

# Force ARM toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Set ARM toolchain paths
set(CMAKE_C_COMPILER /Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER /Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER /Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-gcc)
set(CMAKE_OBJCOPY /Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-objcopy)
set(CMAKE_SIZE /Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin/arm-none-eabi-size)

# Set find program mode
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE NEVER)

# Common STM32 settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# DOF Firmware CMake Helper Functions

# Function to define project paths
function(definePaths project_name)
    set(${project_name}_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
    set(${project_name}_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} PARENT_SCOPE)
endfunction()

# Function to define project includes
function(defineProjectIncludes binary_dir)
    include_directories(${binary_dir})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
endfunction()

# Function to add firmware component item
function(add_item component_name directory_name)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${directory_name})
        add_subdirectory(${directory_name})
        message(STATUS "Added component: ${component_name}")
    else()
        message(WARNING "Component directory not found: ${directory_name}")
    endif()
endfunction()

# Function to conditionally add firmware component
function(add_conditional_item condition component_name directory_name)
    if(${condition})
        add_item(${component_name} ${directory_name})
    else()
        message(STATUS "Skipped component: ${component_name} (condition: ${condition})")
    endif()
endfunction()

# Function to create STM32 target with common settings
function(create_stm32_target target_name)
    # Set STM32 flags for target
    target_compile_options(${target_name} PRIVATE ${STM32_FLAGS})

    # Set common include directories
    target_include_directories(${target_name} PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/src/board/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    )
endfunction()

# Function to create firmware executable
function(create_firmware_executable target_name)
    set_target_properties(${target_name} PROPERTIES
        LINK_FLAGS "${STM32_ADDITIONAL_LINKER_FLAGS} -T${CMAKE_SOURCE_DIR}/STM32F411XE_FLASH.ld --specs=nosys.specs -Wl,-Map=${target_name}.map"
    )

    # Add STM32 system sources
    target_sources(${target_name} PRIVATE
        ${CMAKE_SOURCE_DIR}/src/board/Src/system_stm32f4xx.c
        ${CMAKE_SOURCE_DIR}/src/board/Src/syscalls.c
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_stm32f411xe.s
    )

    # Link standard libraries
    target_link_libraries(${target_name} PRIVATE -lstdc++ -lsupc++)
endfunction()
# STM32 specific flags
set(STM32_FLAGS
    -mcpu=cortex-m4
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -Wall
    -Wextra
    -Wpedantic
    -fdata-sections
    -ffunction-sections
    -fno-rtti
    -fno-exceptions
    -fno-threadsafe-statics
)

# Debug/Release specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND STM32_FLAGS -O0 -g3)
    add_definitions(-DDEBUG)
else()
    list(APPEND STM32_FLAGS -O2 -g)
endif()

# Common STM32 definitions
add_definitions(
    -DUSE_HAL_DRIVER
    -DSTM32F411xE
    -DSTM32_THREAD_SAFE_STRATEGY=2
)

# Function to create STM32 executable
function(create_stm32_executable TARGET_NAME)
    set(oneValueArgs COMPONENT)
    set(multiValueArgs SOURCES LIBS)
    cmake_parse_arguments(OPTIONS "" "${oneValueArgs}" "" "${multiValueArgs}" ${ARGN})
    
    add_executable(${TARGET_NAME} ${OPTIONS_SOURCES})
    
    # Set STM32 flags
    target_compile_options(${TARGET_NAME} PRIVATE ${STM32_FLAGS})
    
    # Set linker script
    set_target_properties(${TARGET_NAME} PROPERTIES
        LINK_FLAGS "-T${CMAKE_SOURCE_DIR}/STM32F411XX_FLASH.ld --specs=nano.specs -Wl,-Map=${TARGET_NAME}.map -Wl,--gc-sections"
    )
    
    # Link libraries
    if(OPTIONS_LIBS)
        target_link_libraries(${TARGET_NAME} ${OPTIONS_LIBS})
    endif()
    
    # Link STM32 specific libraries
    target_link_libraries(${TARGET_NAME}
        -lc -lm
        -lstdc++ -lsupc++
    )
    
    # Set include directories
    target_include_directories(${TARGET_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/src/Core/Inc
        ${CMAKE_SOURCE_DIR}/src/servos/types
        ${CMAKE_SOURCE_DIR}/src/servos/controllers
        ${CMAKE_SOURCE_DIR}/src/servos/configs
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    )
endfunction()

# Function to create STM32 static library
function(create_stm32_library TARGET_NAME)
    set(oneValueArgs COMPONENT)
    set(multiValueArgs SOURCES LIBS)
    cmake_parse_arguments(OPTIONS "" "${oneValueArgs}" "" "${multiValueArgs}" ${ARGN})
    
    add_library(${TARGET_NAME} STATIC ${OPTIONS_SOURCES})
    
    # Set STM32 flags
    target_compile_options(${TARGET_NAME} PRIVATE ${STM32_FLAGS})
    
    # Set include directories
    target_include_directories(${TARGET_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/types
        ${CMAKE_CURRENT_SOURCE_DIR}/controllers
        ${CMAKE_CURRENT_SOURCE_DIR}/configs
        ${CMAKE_SOURCE_DIR}/src/Core/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
        ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
    )
    
    # Link libraries
    if(OPTIONS_LIBS)
        target_link_libraries(${TARGET_NAME} ${OPTIONS_LIBS})
    endif()
    
    # Set C++ standard
    target_compile_features(${TARGET_NAME} PUBLIC cxx_std_20)
endfunction()

# Function to add STM32 HAL sources
function(add_stm32_hal_sources TARGET_NAME)
    target_sources(${TARGET_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
        ${CMAKE_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    )
endfunction()

# Function to add STM32 system sources
function(add_stm32_system_sources TARGET_NAME)
    target_sources(${TARGET_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/src/Core/Src/system_stm32f4xx.c
        ${CMAKE_SOURCE_DIR}/src/Core/Src/stm32f4xx_it.c
        ${CMAKE_SOURCE_DIR}/src/Core/Src/stm32f4xx_hal_msp.c
        ${CMAKE_SOURCE_DIR}/src/Core/Src/syscalls.c
        ${CMAKE_SOURCE_DIR}/src/Core/Src/sysmem.c
        ${CMAKE_SOURCE_DIR}/startup_stm32f411xe.s
    )
endfunction()
