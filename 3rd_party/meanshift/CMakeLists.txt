cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(meanshift)

set (CMAKE_BUILD_TYPE Release)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ../lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

### ----------------------------------------------------------------------------
### Sources

set(MEANSHIFT_SOURCES 
  src/MeanShift.cpp
  src/PointSet.cpp
  src/VectorPointSet.cpp)

message(STATUS "${MEANSHIFT_SOURCES}")
add_library(meanshift SHARED ${MEANSHIFT_SOURCES})
target_include_directories (meanshift PUBLIC include)

install(TARGETS meanshift
        LIBRARY DESTINATION lib
        PUBLIC_HEADER DESTINATION include)


install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include DESTINATION ${CMAKE_INSTALL_PREFIX} )