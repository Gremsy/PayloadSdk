cmake_minimum_required(VERSION 3.5)

project (PayloadSDK)


# # build gSDK lib
set(GSDK_NAME gSDK)
set(GSDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/${GSDK_NAME})
add_subdirectory(${GSDK_DIR} ${CMAKE_CURRENT_BINARY_DIR}/${GSDK_NAME})

#Include header files
include_directories(
            ${GST_INCLUDE_DIRS}
            ${CAIRO_INCLUDE_DIRS}
            ${OPENTLD_INCLUDES}
            ${OpenCV_INCLUDE_DIRS}
            ${CF_HEADER_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include
            ${GSDK_DIR}/src
            ${GSDK_DIR}/src/mavlink/include/mavlink/v2.0
        )

file(GLOB_RECURSE SOURCES 
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/src/*.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/payloadSdkInterface.cpp 
            ${GSDK_DIR}/src/*.cpp         
            )

# PayloadSDK library build
add_library(${PROJECT_NAME} STATIC ${SOURCES} ${GSDK_NAME}) # Static by default
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
    PUBLIC 
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${GST_INCLUDE_DIRS}
            ${CAIRO_INCLUDE_DIRS}
            ${OPENTLD_INCLUDES}
            ${OpenCV_INCLUDE_DIRS}
            ${CF_HEADER_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/
            ${CMAKE_CURRENT_SOURCE_DIR}/third-party/mavlink/include
            ${GSDK_DIR}/src
            ${GSDK_DIR}/src/mavlink/include/mavlink/v2.0
        )