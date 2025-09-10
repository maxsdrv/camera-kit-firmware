# DOF Gimbal Control Component

add_library(dof-gimbal STATIC
    pca9685_servo.cpp
)

create_stm32_target(dof-gimbal)

# Set component-specific include directories
target_include_directories(dof-gimbal PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

# Link with HAL library
target_link_libraries(dof-gimbal PUBLIC stm32_hal)
