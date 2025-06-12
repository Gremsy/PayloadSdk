cmake_minimum_required(VERSION 3.5)

# Project name
project(parseThermalStream)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(PkgConfig REQUIRED)
find_package(ZLIB REQUIRED)
#Use Pkg-config to configure GStreamer
pkg_search_module(GLIB REQUIRED glib-2.0)
pkg_search_module(GLIB REQUIRED glib-2.0)

find_package( OpenCV REQUIRED )
if(OpenCV_FOUND)
    message("Found OpenCV")
    message("Includes: " ${OpenCV_INCLUDE_DIRS})
endif(OpenCV_FOUND)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}
                    ${OpenCV_INCLUDE_DIRS}
                    ${ZLIB_INCLUDE_DIRS}
                    )

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}
                        pthread
                        ${OpenCV_LIBS}
                        ${ZLIB_LIBRARIES}
                        )