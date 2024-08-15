##################### STANDARD HEADER #########################

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

##################### PROJECT DEF #############################
project(mavlinkInterface)

####################### INCLUDE   #############################

find_package(Threads REQUIRED)
#find_package(PkgConfig)
#pkg_search_module(GLIB REQUIRED glib-2.0)
#pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
#pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
#pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

#if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
#  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/../bin")
#endif()

include_directories (${PROJECT_SOURCE_DIR}/inc
                     ${PROJECT_SOURCE_DIR}/inc/mavlink/v2.0)

#add_compile_options(-Wall -Werror -Wextra)
if (QC_PLATFORM)
# https://gcc.gnu.org/bugzilla/show_bug.cgi?id=47037
add_compile_options(-Wno-address-of-packed-member -Wno-cast-align -fcaller-saves)
else()
add_compile_options(-Wno-address-of-packed-member -Wno-cast-align)
endif(QC_PLATFORM)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") # thread

#######################  COMPILE  #############################
file(GLOB SOURCES "src/*.cpp")

# Library build
add_library(${PROJECT_NAME} STATIC ${SOURCES}) # Static by default
target_include_directories(${PROJECT_NAME} PUBLIC
                           ${PROJECT_SOURCE_DIR}/inc
                           ${PROJECT_SOURCE_DIR}/inc/mavlink/v2.0)
