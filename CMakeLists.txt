cmake_minimum_required(VERSION 3.14)

project(ORB_SLAM3)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

# Check C++17 support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
   message(STATUS "Using flag -std=c++17.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV 4.4 REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(Pangolin 0.6 REQUIRED)
find_package(G2O REQUIRED)

set(G2O_LIBRARIES
   ${G2O_CORE_LIBRARY}
   ${G2O_SOLVER_EIGEN}
   ${G2O_SOLVER_DENSE}
   ${G2O_TYPES_DATA}
   ${G2O_TYPES_SBA}
   ${G2O_TYPES_SIM3}
   ${G2O_TYPES_SLAM3D}
)

include_directories(
   ${PROJECT_SOURCE_DIR}
   ${PROJECT_SOURCE_DIR}/include
   ${PROJECT_SOURCE_DIR}/include/CameraModels
   ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
   ${EIGEN3_INCLUDE_DIR}
   ${Pangolin_INCLUDE_DIRS}
   ${G2O_INCLUDE_DIR}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
file(GLOB ${PROJECT_NAME}_SRCS
   ${PROJECT_SOURCE_DIR}/src/*.cc
   ${PROJECT_SOURCE_DIR}/src/*.cpp
   ${PROJECT_SOURCE_DIR}/src/CameraModels/*.cpp
)
file(GLOB ${PROJECT_NAME}_HDRS
   ${PROJECT_SOURCE_DIR}/include/*.h
   ${PROJECT_SOURCE_DIR}/include/CameraModels/*.h
)

add_library(${PROJECT_NAME} SHARED
   ${${PROJECT_NAME}_SRCS}
   ${${PROJECT_NAME}_HDRS}
)

target_link_libraries(${PROJECT_NAME}
   ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
   ${OpenCV_LIBS}
   ${EIGEN3_LIBS}
   ${Pangolin_LIBRARIES}
   ${G2O_LIBRARIES}
   boost_serialization
   crypto
   fmt
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D)
add_executable(rgbd_sim ${PROJECT_SOURCE_DIR}/Examples/RGB-D/rgbd_sim.cc)
target_link_libraries(rgbd_sim ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial)
add_executable(rgbd_inertial_sim ${PROJECT_SOURCE_DIR}/Examples/RGB-D-Inertial/rgbd_inertial_sim.cc)
target_link_libraries(rgbd_inertial_sim ${PROJECT_NAME})