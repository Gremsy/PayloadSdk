cmake_minimum_required(VERSION 3.5)

project(ghadron_project LANGUAGES CXX)


# Source
set(APP_SOURCES
	main.cpp
	)

# Include libs
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
add_executable(${PROJECT_NAME}
               ${APP_SOURCES}
               )
find_package( Threads )
target_link_libraries(${PROJECT_NAME}
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES} 
                     stdc++fs
                     )

install(TARGETS
        ${PROJECT_NAME})

