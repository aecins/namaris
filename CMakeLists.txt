cmake_minimum_required(VERSION 3.0)

project(CppTools)

if(POLICY CMP0053)
    cmake_policy(SET CMP0053 OLD)
endif()

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})
include(install_package)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/sample_pcl_viewer")
add_library(${CMAKE_PROJECT_NAME} INTERFACE)
target_include_directories(${CMAKE_PROJECT_NAME} INTERFACE "include")

#------------- config for install ------------------------------#
set(CppTools_INCLUDE_DIR "include")

set( CONFIG_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set( CONFIG_DIR ${CMAKE_CURRENT_BINARY_DIR})
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/CppToolsConfig.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/CppToolsConfig.cmake @ONLY)

install_package(
        PKG_NAME CppTools
        VERSION 0.1
        DESCRIPTION "C++ utilities for 3D computer vision."
        INSTALL_INCLUDE_DIR true
        DESTINATION ${CMAKE_INSTALL_PREFIX}
)

# uninstall target
include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)

add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)