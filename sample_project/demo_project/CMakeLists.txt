cmake_minimum_required(VERSION 3.5)

project(demo_project LANGUAGES CXX)


###source for project
set(APP_SOURCES
	main.cpp
# 	${SOURCES}
	)



#include 
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${PAYLOADSDK_LIB}
                    ${PAYLOADSDK_LIB}/payload-define
                    ${PAYLOADSDK_DIR}/third-party/mavlink/include
                    ${PAYLOADSDK_DIR}/gSDK/src/mavlink/include/mavlink/v2.0
                    ${PAYLOADSDK_DIR}/gSDK/src
                    )
# 
add_executable(${PROJECT_NAME}
               ${APP_SOURCES}
               )
find_package( Threads )
target_link_libraries(${PROJECT_NAME}
                     ${PAYLOADSDK_LIB}/libPayloadSDK.a
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     stdc++fs
#                      config++ 
                     )

