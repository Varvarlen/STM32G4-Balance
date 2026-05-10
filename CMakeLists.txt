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


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME STM32G431Demo)

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
add_executable(${CMAKE_PROJECT_NAME}
        Core/Src/ws2812b.c
        Core/Inc/ws2812b.h
        Core/Src/buzzer.c
        Core/Inc/buzzer.h
        Core/Inc/comm.h
        Core/Src/comm.c
        Core/Inc/mt6701.h
        Core/Src/mt6701.c
        Core/Inc/encoder_cache.h
        Core/Inc/ina240.h
        Core/Src/ina240.c
        Core/Inc/mpu6050.h
        Core/Src/mpu6050.c
        Core/Inc/kf_angle.h
        Core/Src/kf_angle.c
        Core/Inc/comm_protocol.h
        Core/Src/comm_protocol.c
        Core/Inc/foc.h
        Core/Src/foc.c
        Core/Inc/motor_hal.h
        Core/Src/motor_hal.c
        Core/Inc/transform.h
        Core/Src/transform.c
        Core/Inc/pi.h
        Core/Src/pi.c
        Core/Inc/svpwm.h
        Core/Src/svpwm.c
        Core/Inc/six_step.h
        Core/Src/six_step.c
        Core/Inc/current_ctrl.h
        Core/Src/current_ctrl.c
        Core/Inc/fast_math.h
        Core/Src/fast_math.c
        Core/Inc/calibration.h
        Core/Src/calibration.c
        Core/Inc/debug_capture.h
        Core/Src/debug_capture.c
        Core/Inc/speed_ctrl.h
        Core/Src/speed_ctrl.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
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