cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME Motor_Spin_Minimal)
project(${CMAKE_PROJECT_NAME})
enable_language(C ASM)

set(PARENT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_executable(${CMAKE_PROJECT_NAME})

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H743xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${PARENT_DIR}/Inc
    ${PARENT_DIR}/Utils
    ${PARENT_DIR}/BSP/Inc/GO-M8010-6
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${PARENT_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${PARENT_DIR}/Drivers/CMSIS/Include
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Src/main.c
    ${PARENT_DIR}/startup_stm32h743xx.s
    ${PARENT_DIR}/Src/system_stm32h7xx.c
    ${PARENT_DIR}/Src/sysmem.c
    ${PARENT_DIR}/Src/syscalls.c
    ${PARENT_DIR}/BSP/Src/GO-M8010-6/GO-M8010-6.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c
    ${PARENT_DIR}/Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c
)

target_link_libraries(${CMAKE_PROJECT_NAME} m)

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
    ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map
)

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -T${PARENT_DIR}/STM32H743XG_FLASH.ld
    --specs=nano.specs
    -Wl,-Map=${CMAKE_PROJECT_NAME}.map
    -Wl,--gc-sections
    -Wl,--print-memory-usage
)
