cmake_minimum_required(VERSION 3.5)

project(demo_project LANGUAGES CXX)


###source for project
set(APP_SOURCES
	main.cpp
# 	${SOURCES}
	)

set(PAYLOADSDKLIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../libs)

#include 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${PAYLOADSDKLIB_DIR}
                    ${PAYLOADSDKLIB_DIR}/third-party/mavlink/include
                    ${PAYLOADSDKLIB_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
                    ${PAYLOADSDKLIB_DIR}/gSDK/src
                    )
# 
add_executable(${PROJECT_NAME}
               ${APP_SOURCES}
               )
find_package( Threads )
target_link_libraries(${PROJECT_NAME}
                     ${PAYLOADSDKLIB_DIR}/libPayloadSDK.a
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     stdc++fs
#                      config++ 
                     )

