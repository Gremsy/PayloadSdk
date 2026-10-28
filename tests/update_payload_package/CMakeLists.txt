cmake_minimum_required(VERSION 3.5)

project(payload_updater LANGUAGES CXX)

# Source
set(APP_SOURCES
	main.cpp
	ssh_transfer.cpp
	ota_monitor.cpp
	terminal_ui.cpp
	)

add_executable(${PROJECT_NAME}
               ${APP_SOURCES}
               )
find_package( Threads )
target_link_libraries(${PROJECT_NAME}
                     PayloadSDK
                     ${CMAKE_THREAD_LIBS_INIT}
                     ${GST_LIBRARIES}
                     )
