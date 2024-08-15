##################### STANDARD HEADER #########################

cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

##################### PROJECT DEF #############################
project(mavlink-test)

####################### INCLUDE   #############################
find_package(Threads REQUIRED)

####################  ENV VARIABLES  ##########################
set(LIB_MAVLINK mavlinkInterface)

#######################  COMPILE  #############################

#add_compile_options(-Wall -Werror -Wextra)
add_compile_options(-Wno-address-of-packed-member -Wno-cast-align)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread") # thread

# Executable build
add_executable(${PROJECT_NAME} mavlinklib_test.cpp)
target_link_libraries(${PROJECT_NAME} ${LIB_MAVLINK})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR})
