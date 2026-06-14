SET(CMAKE_LIBRARY_NAME device_driver)

message(STATUS "Building device driver library: ${CMAKE_LIBRARY_NAME}).")

# 配置头文件集合以及源文件集合, 使用CONFIGURE_DEPENDS参数使CMake可以动态识别文件变化.
file(GLOB SOURCE_FILES CONFIGURE_DEPENDS "source/*.c")
file(GLOB HEADER_FILES CONFIGURE_DEPENDS "include/*.h")

# 控制器保留完整安全检查，同时单独优化代码体积以适配 64KB Flash。
set_source_files_properties(source/lqr_controller.c PROPERTIES COMPILE_OPTIONS "-Os")

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

