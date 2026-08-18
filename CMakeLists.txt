cmake_minimum_required(VERSION 3.10)
project(gps_degradation_plugin)

# Set compiler requirements
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Gazebo Sim Components
find_package(gz-sim8 REQUIRED) # Use gz-sim7 if working on Garden, gz-sim8 for Harmonic

add_library(GpsDegradationPlugin SHARED GpsDegradationPlugin.cc)

target_link_libraries(GpsDegradationPlugin
  PRIVATE gz-sim8::core
)

# Set clean output directories for installation target scopes
install(TARGETS GpsDegradationPlugin DESTINATION ${PROJECT_BINARY_DIR}/lib)
