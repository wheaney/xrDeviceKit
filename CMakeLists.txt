cmake_minimum_required(VERSION 3.16)
project(xrDeviceKit C)

set(CMAKE_C_STANDARD 17)

find_package(json-c REQUIRED CONFIG)

add_subdirectory(modules/Fusion/Fusion)

add_library(
		xrDeviceKit
		src/device_imu.c
		src/device_mcu.c
)

target_compile_options(xrDeviceKit PRIVATE -fPIC)

target_include_directories(xrDeviceKit
		BEFORE PUBLIC include
)

target_include_directories(xrDeviceKit
		SYSTEM BEFORE PRIVATE
		${CMAKE_CURRENT_SOURCE_DIR}/modules/Fusion
)

target_link_libraries(xrDeviceKit
		PRIVATE json-c::json-c Fusion m
)

set(XRDEVICEKIT_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" PARENT_SCOPE)
set(XRDEVICEKIT_LIBRARY xrDeviceKit PARENT_SCOPE)
