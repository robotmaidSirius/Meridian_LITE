add_library(lib_meridian_plugin_on_picoSdk STATIC
	${CMAKE_CURRENT_LIST_DIR}/src/board/meridian_board_lite_for_pico_w.cpp
	${CMAKE_CURRENT_LIST_DIR}/src/board/fireflake_base_board_for_pico_w.cpp
)

target_link_libraries(lib_meridian_plugin_on_picoSdk
    pico_stdlib
)

target_include_directories(lib_meridian_plugin_on_picoSdk PUBLIC
	${CMAKE_CURRENT_LIST_DIR}/include
)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D BUILD_TYPE=\\\"Debug\\\" -D ENABLE_LOG_INFO=1 -D ENABLE_LOG_ERROR=1 -D ENABLE_LOG_DEBUG=1")
	message(STATUS "# Build \"lib_meridian_plugin_on_picoSdk\":Debug")
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D BUILD_TYPE=\\\"BUILD_TYPE=Release\\\"")
	message(STATUS "# Build \"lib_meridian_plugin_on_picoSdk\":Release")
endif()

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../Meridian_Core
    Meridian_Core)

target_link_libraries(lib_meridian_plugin_on_picoSdk
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
		pico_btstack_classic
		pico_btstack_ble
		pico_btstack_cyw43
	pico_multicore
	lib_meridian_core
)
