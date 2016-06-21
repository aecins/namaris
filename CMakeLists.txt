cmake_minimum_required(VERSION 2.8)

project(CppTools)

if(POLICY CMP0053)
    cmake_policy(SET CMP0053 OLD)
endif()

set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

# Install 3rd party
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/3rd_party")

# Install library
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION ${CMAKE_INSTALL_PREFIX} )

# uninstall target
include(${CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in)
add_custom_target(uninstall
        COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)