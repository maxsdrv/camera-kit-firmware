# DOF Telemetry Component

add_library(dof-telemetry STATIC
    telemetry_system.cpp
)

create_stm32_target(dof-telemetry)

# Set component-specific include directories
target_include_directories(dof-telemetry PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with HAL library
target_link_libraries(dof-telemetry PUBLIC stm32_hal)
