CMAKE_MINIMUM_REQUIRED (VERSION 2.8.8)
PROJECT( namaris )

# version (e.g. for packaging)
set (NAMARIS_MAJOR_VERSION 0)
set (NAMARIS_MINOR_VERSION 1)
set (NAMARIS_PATCH_VERSION 0)
set (NAMARIS_VERSION ${NAMARIS_MAJOR_VERSION}.${NAMARIS_MINOR_VERSION}.${NAMARIS_PATCH_VERSION})
if (COMMAND cmake_policy)
  cmake_policy (SET CMP0003 NEW)
endif (COMMAND cmake_policy)

# Additional CMake modules
SET (CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# Set include directories
set (INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES (${INCLUDE_DIRS})

# Installation
install (
  DIRECTORY ${PROJECT_SOURCE_DIR}/include/namaris/
  DESTINATION include/namaris
  FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h" )

# uninstall target (this does not delete the directories created during installation)
configure_file (
    "${PROJECT_SOURCE_DIR}/CMakeModules/CMakeUninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY )

add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE namaris)

# Create a octomap-config.cmake file for the use from the build tree
set(NAMARIS_INCLUDE_DIRS "${INCLUDE_DIRS}")

include(CMakePackageConfigHelpers)

CONFIGURE_PACKAGE_CONFIG_FILE (
  namaris-config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/namaris-config.cmake"
  PATH_VARS NAMARIS_INCLUDE_DIRS
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/namaris)

WRITE_BASIC_PACKAGE_VERSION_FILE (
  "${CMAKE_CURRENT_BINARY_DIR}/namaris-config-version.cmake"
  VERSION ${NAMARIS_VERSION}
  COMPATIBILITY AnyNewerVersion )

# Create a octomap-config.cmake file for the use from the install tree
# and install it
set(NAMARIS_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")

CONFIGURE_PACKAGE_CONFIG_FILE(
  namaris-config.cmake.in
  "${PROJECT_BINARY_DIR}/InstallFiles/namaris-config.cmake"
  PATH_VARS NAMARIS_INCLUDE_DIRS
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/namaris)

WRITE_BASIC_PACKAGE_VERSION_FILE(
  ${PROJECT_BINARY_DIR}/InstallFiles/namaris-config-version.cmake
  VERSION ${NAMARIS_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES
  "${PROJECT_BINARY_DIR}/InstallFiles/namaris-config.cmake"
  "${PROJECT_BINARY_DIR}/InstallFiles/namaris-config-version.cmake" 
  DESTINATION share/namaris/)

# Write pkgconfig-file:
include (InstallPkgConfigFile)
install_pkg_config_file (namaris
    CFLAGS
    REQUIRES
    VERSION ${OCTOMAP_VERSION})
