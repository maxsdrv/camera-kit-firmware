# DOF Firmware Source Components

option(WITH_CAMERA_CONTROL "Enable camera control system" OFF)
option(WITH_COMMUNICATION "Enable communication system" OFF)
option(WITH_TELEMETRY "Enable telemetry system" OFF)
option(WITH_GIMBAL_CONTROL "Enable gimbal control system" OFF)

# Core components - always built
add_item(dof-board board)
add_item(dof-main main)

# Conditional components
add_conditional_item(WITH_CAMERA_CONTROL dof-camera-control camera)
add_conditional_item(WITH_COMMUNICATION dof-communication communication)
add_conditional_item(WITH_TELEMETRY dof-telemetry telemetry)
add_conditional_item(WITH_GIMBAL_CONTROL dof-gimbal-control gimbal)

# Create the main firmware executable
add_executable(dof-firmware
    main.cpp
)

create_stm32_target(dof-firmware)
create_firmware_executable(dof-firmware)

# Link with HAL library
target_link_libraries(dof-firmware PUBLIC stm32_hal)

# Conditionally link component libraries
if(WITH_CAMERA_CONTROL)
    target_link_libraries(dof-firmware PRIVATE dof-camera-control)
endif()

if(WITH_COMMUNICATION)
    target_link_libraries(dof-firmware PRIVATE dof-communication)
endif()

if(WITH_TELEMETRY)
    target_link_libraries(dof-firmware PRIVATE dof-telemetry)
endif()

if(WITH_GIMBAL_CONTROL)
    target_link_libraries(dof-firmware PRIVATE dof-gimbal-control)
endif()
