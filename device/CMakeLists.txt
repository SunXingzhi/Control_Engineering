SET(CMAKE_LIBRARY_NAME device_driver)

add_library(${CMAKE_LIBRARY_NAME} STATIC
	driver_step_motor.c
	driver_step_motor.h
	test_step_motor.c
	test_step_motor.h
	public_rule.c
	public_rule.h
	uart.c
	uart.h
	mt6701.c
	mt6701.h
	device_err.h)
message(STATUS "Building device driver library: ${CMAKE_LIBRARY_NAME}).")

file(GLOB SOURCE_FILES CONFIGURE_DEPENDS *.c)
file(GLOB HEADER_FILES CONFIGURE_DEPENDS *.h)

message(STATUS "Source files list: ${SOURCE_FILES}, header files list: ${HEADER_FILES}")

if ((NOT SOURCE_FILES) AND (NOT HEADER_FILES))
	message(WARNING "device driver library:${CMAKE_LIBRARY_NAME} is empty.")
endif ()

target_include_directories(${CMAKE_LIBRARY_NAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR})

# Link another libraries. such as HAL
target_link_libraries(device_driver PUBLIC stm32cubemx)

