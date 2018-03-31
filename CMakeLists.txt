set(PROJECT_NAME ONLINE_CALIB)

project(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.6)

set(BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Required libraries
find_package(OpenCV 2.4 REQUIRED)

# Source files
set(ONLINE_CALIB_SOURCE_FILES
        ${PROJECT_SOURCE_DIR}/src/Database.cpp
        ${PROJECT_SOURCE_DIR}/src/GainRobustTracker.cpp
        ${PROJECT_SOURCE_DIR}/src/ImageReader.cpp
        ${PROJECT_SOURCE_DIR}/src/JacobianGenerator.cpp
        ${PROJECT_SOURCE_DIR}/src/NonlinearOptimizer.cpp
        ${PROJECT_SOURCE_DIR}/src/OptimizationBlock.cpp
        ${PROJECT_SOURCE_DIR}/src/RapidExposureTimeEstimator.cpp
        ${PROJECT_SOURCE_DIR}/src/Tracker.cpp
        ${PROJECT_SOURCE_DIR}/src/VignetteModel.cpp

        ${PROJECT_SOURCE_DIR}/src/Database.h
        ${PROJECT_SOURCE_DIR}/src/Feature.h
        ${PROJECT_SOURCE_DIR}/src/Frame.h
        ${PROJECT_SOURCE_DIR}/src/GainRobustTracker.h
        ${PROJECT_SOURCE_DIR}/src/ImageReader.h
        ${PROJECT_SOURCE_DIR}/src/JacobianGenerator.h
        ${PROJECT_SOURCE_DIR}/src/NonlinearOptimizer.h
        ${PROJECT_SOURCE_DIR}/src/OptimizationBlock.h
        ${PROJECT_SOURCE_DIR}/src/OptimizedPoint.h
        ${PROJECT_SOURCE_DIR}/src/RapidExposureTimeEstimator.h
        ${PROJECT_SOURCE_DIR}/src/ResponseModel.h
        ${PROJECT_SOURCE_DIR}/src/StandardIncludes.h
        ${PROJECT_SOURCE_DIR}/src/Tracker.h
        ${PROJECT_SOURCE_DIR}/src/VignetteModel.h
)

include_directories(${PROJECT_SOURCE_DIR}/src)

# Compile main library
add_library(online_calib ${ONLINE_CALIB_SOURCE_FILES})

message("--- Compiling online_calib_demo.")
add_executable(online_calib_demo ${PROJECT_SOURCE_DIR}/src/main.cpp)
target_link_libraries(online_calib_demo
        online_calib
        ${OpenCV_LIBS})