# Simulator build (runs on your PC, no ESP32 needed).
#
# Visual Studio:  File > Open > Folder... pick this folder, then choose a
#                 sim_test_*.exe from the "Select Startup Item" dropdown and press Run.
# Command line:   cmake -S . -B build  &&  cmake --build build  &&  ./build/sim_test_imu
#
# The ESP32 build is separate: see platformio.ini.

cmake_minimum_required(VERSION 3.16)
project(SelfBalancingRobotSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Code shared by every simulator program
add_library(sbr_sim STATIC
	src/hal/sim/RobotPhysics.cpp
	src/hal/sim/SimHardware.cpp
	src/comms/sim/ScriptedLink.cpp
	src/devices/MPU6050.cpp
	src/devices/DcMotor.cpp
	src/devices/HCSR04.cpp
	src/apps/TestReport.cpp
)
target_include_directories(sbr_sim PUBLIC src include)
target_compile_definitions(sbr_sim PUBLIC SBR_SIM)
if(MSVC)
	target_compile_options(sbr_sim PUBLIC /W4 /D_CRT_SECURE_NO_WARNINGS)
else()
	target_compile_options(sbr_sim PUBLIC -Wall -Wextra)
endif()

# One program per app (the same app files the ESP32 runs)
foreach(app test_imu test_motors test_ultrasonic test_bluetooth)
	add_executable(sim_${app} src/main/sim_main.cpp src/apps/${app}.cpp)
	target_link_libraries(sim_${app} PRIVATE sbr_sim)
endforeach()
