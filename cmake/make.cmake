# STM32 Project Build System

cmake_minimum_required(VERSION 3.22)

# Set policies
if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

# Force ARM toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR ARM)

# Set ARM toolchain paths (use environment variables if available)
if(NOT DEFINED ARM_TOOLCHAIN_PATH)
    set(ARM_TOOLCHAIN_PATH "/Applications/ArmGNUToolchain/14.2.rel1/arm-none-eabi/bin")
endif()

set(CMAKE_C_COMPILER ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-gcc)
set(CMAKE_OBJCOPY ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-objcopy)
set(CMAKE_SIZE ${ARM_TOOLCHAIN_PATH}/arm-none-eabi-size)

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

# STM32 specific flags
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

# Debug/Release specific flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND STM32_FLAGS -O0 -g3)
else()
    list(APPEND STM32_FLAGS -O2 -g)
endif()

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

# Function to create STM32 target with common settings
function(create_stm32_target target_name)
    # Set STM32 flags for target
    target_compile_options(${target_name} PRIVATE ${STM32_FLAGS})

    # Set common include directories
    target_include_directories(${target_name} PRIVATE
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
        LINK_FLAGS "-T${CMAKE_SOURCE_DIR}/STM32F411XE_FLASH.ld --specs=nosys.specs -Wl,-Map=${target_name}.map -Wl,--gc-sections"
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