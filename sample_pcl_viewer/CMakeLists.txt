cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(sample_pcl_viewer)
set (CMAKE_BUILD_TYPE Debug)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/../bin")
add_definitions("-Wall -pedantic")

### ----------------------------------------------------------------------------
### Add CPP tools
### ----------------------------------------------------------------------------

set (CPP_TOOLS_INCLUDE_DIRS "../")    # Make sure this points to cpp_tools directory
include_directories (${CPP_TOOLS_INCLUDE_DIRS})

### ----------------------------------------------------------------------------
### PCL dependent options
### ----------------------------------------------------------------------------

find_package(PCL REQUIRED)
message (STATUS "")
if (PCL_FOUND)
  message (STATUS "")
  message (STATUS " -----------------------")
	message (STATUS " Found PCL version ${PCL_VERSION}")
  message (STATUS "     * include dirs: ${PCL_INCLUDE_DIRS}")
  message (STATUS "     * library dirs: ${PCL_LIBRARY_DIRS}")
  message (STATUS "     * libraries:    ${PCL_LIBRARIES}")
else()
	message (FATAL_ERROR : "PCL NOT FOUND!")
endif()
message (STATUS "")

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

### ----------------------------------------------------------------------------
### BOOST
### ----------------------------------------------------------------------------

find_package(Boost REQUIRED COMPONENTS regex) 

if(Boost_FOUND)
  message (STATUS " --------------------------")
  message (STATUS " Found Boost version ${Boost_VERSION}")
  message (STATUS "     * include dirs: ${Boost_INCLUDE_DIRS}")
  message (STATUS "     * libraries:    ${Boost_LIBRARIES}")
endif()

### ----------------------------------------------------------------------------
### Project dependencies
### ----------------------------------------------------------------------------

set (includes ./include
              ../cpp_tools)
include_directories (${includes})

### ----------------------------------------------------------------------------
### C++11 support
### ----------------------------------------------------------------------------

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

### ----------------------------------------------------------------------------
### Executables
### ----------------------------------------------------------------------------

add_executable (sample_pcl_viewer src/main.cpp )
target_link_libraries (sample_pcl_viewer ${PCL_LIBRARIES} ${Boost_LIBRARIES})