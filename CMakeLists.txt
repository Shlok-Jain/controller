cmake_minimum_required(VERSION 3.5)
project(nmpc_project)

# Set the C++ standard to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find the ACADO toolkit
find_package(ACADO REQUIRED)

# Find Eigen3 and Boost (needed for ACADO)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

# Include directories for ACADO, Eigen, and Boost
include_directories(
  ${ACADO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Add the executable for your NMPC code
add_executable(nmpc_controller src/nmpc_controller.cpp)

# Link the necessary libraries (ACADO, Eigen, and Boost)
target_link_libraries(nmpc_controller
  ${ACADO_SHARED_LIBRARIES}
  ${Boost_LIBRARIES}
  Eigen3::Eigen
)

