cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(meanshift)

set (CMAKE_BUILD_TYPE Debug)
# set (CMAKE_BUILD_TYPE Release)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ../bin)
set (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ../lib)
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ../lib)

### ----------------------------------------------------------------------------
### Other library depdendencies
### ----------------------------------------------------------------------------

set (includes 	./include
				../geometry/include)
				# ../CLAPACK/include)
include_directories (${includes})

### ----------------------------------------------------------------------------
### Libraries
### ----------------------------------------------------------------------------

set(MEANSHIFT_SOURCES "src/MeanShift.cpp;src/PointSet.cpp;src/VectorPointSet.cpp")

# file(GLOB MEANSHIFT_SOURCES "src/*.cpp")
message(STATUS "${MEANSHIFT_SOURCES}")
add_library(meanshift ${MEANSHIFT_SOURCES})
target_link_libraries(meanshift)