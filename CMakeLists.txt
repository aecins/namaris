cmake_minimum_required(VERSION 2.8)

project(CppTools)

if (POLICY CMP0053)
    cmake_policy(SET CMP0053 OLD)
endif ()
if (POLICY CMP0062)
    cmake_policy(SET CMP0062 OLD)
endif ()


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})
include(install_package)
set(Namaris_INCLUDE_DIR "include")

# Install 3rd party
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party")

# Install library
install_package(
        PKG_NAME Namaris
        VERSION 0.1
        DESCRIPTION "C++ utilities for 3D computer vision."
        INSTALL_INCLUDE_DIR true
        DESTINATION ${CMAKE_INSTALL_PREFIX}
)

# uninstall target
include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)
add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)