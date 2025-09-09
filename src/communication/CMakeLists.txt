# DOF Communication Component

add_library(dof-communication STATIC
    CommunicationManager.cpp
)

create_stm32_target(dof-communication)

# Set component-specific include directories
target_include_directories(dof-communication PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with HAL library
target_link_libraries(dof-communication PUBLIC stm32_hal)
