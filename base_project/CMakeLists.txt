cmake_minimum_required(VERSION 3.0...3.13)

message("CMake the running...\n")

include_directories(inc)

set(SOURCES src/main.c src/debug.c src/fc_init.c src/flight_data.c src/iface.c src/motors.c src/scheduler.c src/tasks src/drone_time)

add_executable(drone_firmware ${SOURCES})

project(drone_firmware)

message("CMake the stop\n")