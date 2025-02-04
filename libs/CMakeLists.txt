cmake_minimum_required(VERSION 3.5)

project (PayloadSDK)

#Include header files
include_directories(
            ${GST_INCLUDE_DIRS}
            ${CF_HEADER_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include
            ${CMAKE_CURRENT_SOURCE_DIR}/payload-define
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include/mavlink/v2.0
        )

file(GLOB_RECURSE SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/src/*.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/payloadSdkInterface.cpp 
            )

# PayloadSDK library build
add_library(${PROJECT_NAME} STATIC ${SOURCES}) # Static by default
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
    PUBLIC 
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${GST_INCLUDE_DIRS}
            ${CF_HEADER_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include
            ${CMAKE_CURRENT_SOURCE_DIR}/payload-define
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include/mavlink/v2.0
        )