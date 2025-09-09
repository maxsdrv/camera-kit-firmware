# DOF Gimbal Control Component

add_library(dof-gimbal-control STATIC
    controllers/pca9685_servo_controller.cpp
    controllers/pwm_servo_controller.cpp
    controllers/dof_robotic_arm.cpp
)

create_stm32_target(dof-gimbal-control)

# Set component-specific include directories
target_include_directories(dof-gimbal-control PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/types
    ${CMAKE_CURRENT_SOURCE_DIR}/controllers
    ${CMAKE_CURRENT_SOURCE_DIR}/configs
)

# Link with HAL library
target_link_libraries(dof-gimbal-control PUBLIC stm32_hal)
