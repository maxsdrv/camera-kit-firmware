# DOF Firmware Board Component

add_library(dof-board STATIC
    Src/stm32f4xx_it.c
    Src/stm32f4xx_hal_msp.c
    Src/syscalls.c
    Src/sysmem.c
)

create_stm32_target(dof-board)

target_link_libraries(dof-board PUBLIC stm32_hal)
