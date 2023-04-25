cmake_minimum_required(VERSION 3.10)

# set the project name
project(test LANGUAGES C ASM)
set(CMAKE_SYSTEM_PROCESSOR "rv32imac")

# Set your SDK ROOT here
set(SDK_ROOT "sdk") 
set(DRIVER_ROOT "${SDK_ROOT}/Peripheral")
set(SDK_CORE_ROOT "${SDK_ROOT}/Core")
set(SDK_INIT_ROOT "${SDK_ROOT}/Startup")
get_filename_component(LINKER_SCRIPT "${SDK_ROOT}/Ld/Link.ld" REALPATH)

set(TARGET main)

# add the executable
add_executable(${TARGET}.elf 
    src/main.c
    src/ch32v10x_it.c
    # src/communication.c
    # src/config.c
    # src/motor_control.c
    # src/pid.c
    # src/batt.c
    
    # src/systick.c
    ${DRIVER_ROOT}/src/ch32v10x_misc.c
    ${DRIVER_ROOT}/src/ch32v10x_gpio.c
    # ${DRIVER_ROOT}/Source/gd32vf103_timer.c
    ${DRIVER_ROOT}/src/ch32v10x_usart.c
    ${DRIVER_ROOT}/src/ch32v10x_rcc.c
    # ${DRIVER_ROOT}/Source/gd32vf103_eclic.c
    ${SDK_CORE_ROOT}/core_riscv.c
    ${SDK_INIT_ROOT}/startup_ch32v10x.S
    ${SDK_ROOT}/Debug/debug.c
    ${SDK_ROOT}/system_ch32v10x.c
)

target_include_directories(${TARGET}.elf PRIVATE 
    src
    ${SDK_ROOT}
    ${SDK_CORE_ROOT}
    ${SDK_ROOT}/Debug
    ${DRIVER_ROOT}
    ${DRIVER_ROOT}/inc
)

target_compile_definitions(${TARGET}.elf PRIVATE 
    # "DEBUG"
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
target_compile_definitions(${TARGET}.elf PRIVATE 
    "DEBUG"
)
endif()

target_compile_options(${TARGET}.elf PRIVATE 
    "-O3"
    "-g"
    "-Wall"
    "-flto"
    "-fshort-enums"
    "-ffunction-sections"
    # "-fdata-sections"
)

target_link_options(${TARGET}.elf PRIVATE 
    "--specs=nosys.specs"
    "--specs=nano.specs"
    "-nostartfiles"
    "-fno-exceptions"
    "-flto"
    "-Xlinker"
    "-T" "${LINKER_SCRIPT}"
    "-Wl,-Map=${TARGET}.map"
    "-Wl,--print-memory-usage"
)

# Post processing command to create a hex file 
add_custom_command(TARGET ${TARGET}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary  ${TARGET}.elf  ${TARGET}.bin
        COMMENT "Invoking: Hexdump")

        