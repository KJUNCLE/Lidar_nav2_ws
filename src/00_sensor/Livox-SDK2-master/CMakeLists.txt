cmake_minimum_required(VERSION 3.0)

project(livox_sdk2)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

message(STATUS "main project dir: " ${PROJECT_SOURCE_DIR})

if (CMAKE_CROSSCOMPILING)
	set(THREADS_PTHREAD_ARG
		"PLEASE_FILL_OUT-FAILED_TO_RUN"
		CACHE STRING "Result from TRY_RUN" FORCE)
endif()

if (UNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif(UNIX)

add_subdirectory(sdk_core)
add_subdirectory(samples)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2ConfigVersion.cmake"
  VERSION 1.3.1
  COMPATIBILITY SameMajorVersion
)

install(EXPORT livox_sdk2Targets
  NAMESPACE livox_sdk2::
  FILE livox_sdk2Targets.cmake
  DESTINATION lib/cmake/livox_sdk2
)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/livox_sdk2Config.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2Config.cmake"
  INSTALL_DESTINATION lib/cmake/livox_sdk2
)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2Config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2ConfigVersion.cmake"
  DESTINATION lib/cmake/livox_sdk2
)
