cmake_minimum_required(VERSION 3.10)
project(gamepad_ctrl CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

# Thu vien gamepad_reader
add_library(gamepad_reader STATIC
  src/gamepad_reader.cpp
)
target_include_directories(gamepad_reader PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_link_libraries(gamepad_reader PUBLIC Threads::Threads)
target_compile_options(gamepad_reader PRIVATE -Wall -Wextra)

# GCC 8 can them thu vien rieng cho std::filesystem
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
  target_link_libraries(gamepad_reader PUBLIC stdc++fs)
endif()

# Chuong trinh chinh su dung thu vien
add_executable(gamepad_ctrl gamepad_ctrl.cpp)
target_link_libraries(gamepad_ctrl PRIVATE gamepad_reader PayloadSDK)
target_compile_options(gamepad_ctrl PRIVATE -Wall -Wextra)