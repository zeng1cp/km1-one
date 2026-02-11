cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-Wno-missing-braces -Wno-missing-field-initializers)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()
# Set the project name
set(CMAKE_PROJECT_NAME km1-one)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# 生成 HEX 文件的规则
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMENT "Generating HEX file: ${CMAKE_PROJECT_NAME}.hex"
)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here

    ### utils
    User/utils/ringbuffer.c

    ### servo
    User/servo/drivers/servo_hal.c
    User/servo/motion/motion_engine.c
    User/servo/motion/motion_sync.c
    User/servo/motion/motion_cycle.c
    User/servo/control/robot_arm_control.c
    
    ### comm - core 核心模块
    
    ### comm - 协议定义
    User/comm/protocol/protocol.c
    User/comm/protocol/state_sender.c
    User/comm/protocol/listener/sys_listener.c
    User/comm/protocol/listener/servo_listener.c
    User/comm/protocol/listener/motion_listener.c
    User/comm/protocol/listener/arm_listener.c
    User/comm/protocol/listener/config_listener.c

    ### comm - drivers 驱动
    User/comm/drivers/uart_driver.c
    
    ### comm - transport 传输层
    User/comm/transport/tf_uart_port.c
    User/comm/transport/TinyFrame/TinyFrame.c
    User/comm/transport/TinyFrame/utils.c
    
    ### comm - dispatcher 分发器
    
    ### comm - api API层
    
    ### comm - utils 工具（可选）
    # User/comm/utils/debug_output.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE

    # Add user defined include paths

    ### utils
    User/utils

    ### servo
    User/servo/drivers
    User/servo/control
    User/servo/motion
    
    ### comm - 核心模块
    User/comm/core
    
    ### comm - 协议定义
    User/comm/protocol
    
    ### comm - 驱动层
    User/comm/drivers

    ### comm - 传输层
    User/comm/transport
    User/comm/transport/TinyFrame
    
    ### comm - 分发器
    
    ### comm - API层
    
    ### comm - 工具（可选）
    # User/comm/utils
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
