SET(TEST_LIST_SOURCES_FILES
	${CMAKE_CURRENT_LIST_DIR}/mrd_communication/i_mrd_conversation.cpp
	${CMAKE_CURRENT_LIST_DIR}/mrd_communication/i_mrd_diagnostic.cpp

	${CMAKE_CURRENT_LIST_DIR}/mrd_plugin/i_mrd_plugin_eeprom.cpp
	${CMAKE_CURRENT_LIST_DIR}/mrd_plugin/i_mrd_plugin_gpio_in_out.cpp
	${CMAKE_CURRENT_LIST_DIR}/mrd_plugin/i_mrd_plugin_i2c.cpp
	${CMAKE_CURRENT_LIST_DIR}/mrd_plugin/i_mrd_plugin_pad.cpp
	${CMAKE_CURRENT_LIST_DIR}/mrd_plugin/i_mrd_plugin_sd.cpp
	${CMAKE_CURRENT_LIST_DIR}/mrd_plugin/i_mrd_plugin_servo.cpp
	${CMAKE_CURRENT_LIST_DIR}/mrd_plugin/i_mrd_plugin_spi.cpp
	${CMAKE_CURRENT_LIST_DIR}/mrd_plugin/i_mrd_plugin.cpp

	${CMAKE_CURRENT_LIST_DIR}/Meridim90.cpp
)
