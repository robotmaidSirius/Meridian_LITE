add_library(lib_meridian_core STATIC
	${CMAKE_CURRENT_LIST_DIR}/src/meridian_core.cpp
)

target_link_libraries(lib_meridian_core
    pico_stdlib
)

target_include_directories(lib_meridian_core PUBLIC
	${CMAKE_CURRENT_LIST_DIR}/include
)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D BUILD_TYPE=\\\"Debug\\\" -D ENABLE_LOG_INFO=1 -D ENABLE_LOG_ERROR=1 -D ENABLE_LOG_DEBUG=1")
	message(STATUS "# Build \"lib_meridian_core\":Debug")
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D BUILD_TYPE=\\\"BUILD_TYPE=Release\\\"")
	message(STATUS "# Build \"lib_meridian_core\":Release")
endif()

target_link_libraries(lib_meridian_core
	hardware_spi
	hardware_i2c
	hardware_dma
	hardware_pio
	hardware_interp
	hardware_timer
	hardware_watchdog
	hardware_clocks
	hardware_gpio
	pico_cyw43_arch_none
	pico_multicore
)
