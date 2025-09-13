# DOF Firmware Source Components

# Core components - always built  
add_item(dof-gimbal gimbal)

# Create the main firmware executable
add_executable(dof-firmware
        main.cpp
        commands.cpp
        interrupts.cpp
        ../Core/Src/stm32f4xx_hal_msp.c
        ../Core/Src/stm32f4xx_it.c
        ../Core/Src/sysmem.c
)

create_stm32_target(dof-firmware)
create_firmware_executable(dof-firmware)

# Link with HAL library and gimbal control
target_link_libraries(dof-firmware PUBLIC stm32_hal dof-gimbal)
