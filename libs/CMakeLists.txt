cmake_minimum_required(VERSION 3.5)

project (PayloadSDK)


# # build gSDK lib
# set(GSDK_NAME gSDK)
# set(GSDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${GSDK_NAME})
# add_subdirectory(${GSDK_DIR} ${CMAKE_CURRENT_BINARY_DIR}/${GSDK_NAME})

set(LIB_MAVLINK mavlinkInterface)
set(MAVLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${LIB_MAVLINK}/libs)
if(EXISTS "${MAVLINK_DIR}")
message("MAVLINK_DIR: FOUND ${MAVLINK_DIR}")
else()
message("MAVLINK_DIR: NOT FOUND ${MAVLINK_DIR}")
endif(EXISTS "${MAVLINK_DIR}")
add_subdirectory(${MAVLINK_DIR} ${CMAKE_CURRENT_BINARY_DIR}/${LIB_MAVLINK}/libs)

#Include header files
include_directories(
            ${GST_INCLUDE_DIRS}
            ${CF_HEADER_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include
            ${CMAKE_CURRENT_SOURCE_DIR}/payload-define
            ${MAVLINK_DIR}/inc
            ${MAVLINK_DIR}/inc/mavlink/v2.0
        )

file(GLOB_RECURSE SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/src/*.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/payloadSdkInterface.cpp 
            )

# PayloadSDK library build
add_library(${PROJECT_NAME} STATIC ${SOURCES} ${LIB_MAVLINK}) # Static by default
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
    PUBLIC 
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${GST_INCLUDE_DIRS}
            ${CF_HEADER_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include
            ${CMAKE_CURRENT_SOURCE_DIR}/payload-define
            ${MAVLINK_DIR}/inc
            ${MAVLINK_DIR}/inc/mavlink/v2.0
        )