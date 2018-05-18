cmake_minimum_required(VERSION 3.2)
set(PROJECT_NAME online_photometric_calibration)

project(${PROJECT_NAME})

# Set default build type if not specified otherwise.
# See https://cmake.org/pipermail/cmake/2012-May/050243.html
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

# We need at least C++11
set(CMAKE_CXX_STANDARD 11)

# set march=native unless specified
if(NOT CXX_MARCH)
  set(CXX_MARCH native)
endif()
SET(CMAKE_CXX_FLAGS_RELEASE "-march=${CXX_MARCH} ${CMAKE_CXX_FLAGS_RELEASE}")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "-march=${CXX_MARCH} ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")

# warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Required libraries
find_package(OpenCV 2.4 REQUIRED)
find_package(Threads REQUIRED)

# Configure CCache if available (requires cmake 3.4 or newer)
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
  set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
  set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
endif(CCACHE_PROGRAM)

# output paths in the build directory
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# include path
include_directories(${PROJECT_SOURCE_DIR}/src)

# Source files
set(ONLINE_CALIB_SOURCE_FILES
  src/Database.cpp
  src/GainRobustTracker.cpp
  src/ImageReader.cpp
  src/JacobianGenerator.cpp
  src/NonlinearOptimizer.cpp
  src/OptimizationBlock.cpp
  src/RapidExposureTimeEstimator.cpp
  src/Tracker.cpp
  src/VignetteModel.cpp
)

# add include files to custom target so they show up in IDEs like
# QtCreator in the project view
file(GLOB_RECURSE _INCLUDE_FILES "src/*.h")
add_custom_target(_include_files_fix_target SOURCES ${_INCLUDE_FILES})

# main library
add_library(online_pcalib SHARED ${ONLINE_CALIB_SOURCE_FILES})

# demo executable for online calibration
add_executable(online_pcalib_demo src/main.cpp)
target_link_libraries(online_pcalib_demo
  online_pcalib
  ${OpenCV_LIBS}
  ${CMAKE_THREAD_LIBS_INIT})

# install rules
install (TARGETS online_pcalib online_pcalib_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
