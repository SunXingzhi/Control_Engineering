SET(CMAKE_LIBRARY_NAME device_driver)

message(STATUS "Building device driver library: ${CMAKE_LIBRARY_NAME}).")

file(GLOB SOURCE_FILES CONFIGURE_DEPENDS "source/*.c")
file(GLOB HEADER_FILES CONFIGURE_DEPENDS "include/*.h")
if ((NOT SOURCE_FILES) AND (NOT HEADER_FILES))
	message(WARNING "device driver library:${CMAKE_LIBRARY_NAME} is empty.")
endif ()

message(STATUS "Source files list: ${SOURCE_FILES}, header files list: ${HEADER_FILES}")
# 添加目标:库文件
add_library(${CMAKE_LIBRARY_NAME} STATIC
	${SOURCE_FILES}
	${HEADER_FILES}
)

target_include_directories(${CMAKE_LIBRARY_NAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include)

# Link another libraries. such as HAL
target_link_libraries(device_driver PUBLIC stm32cubemx)

