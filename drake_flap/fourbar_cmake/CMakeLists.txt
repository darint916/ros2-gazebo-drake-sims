cmake_minimum_required(VERSION 3.16)
project(sdf_sim)

# Enable CTest for testing support.
include(CTest)

# Global compiler and build settings.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find external dependencies.
find_package(Threads MODULE REQUIRED)
find_package(drake CONFIG REQUIRED PATHS /opt/drake)
find_package(Python3 ${drake_PYTHON_VERSION} EXACT REQUIRED COMPONENTS Interpreter Development)

# Compute the Drake Python path for use in Python tests (if needed).
get_filename_component(DRAKE_PYTHONPATH "${drake_DIR}" DIRECTORY)
get_filename_component(DRAKE_PYTHONPATH "${DRAKE_PYTHONPATH}" DIRECTORY)
set(DRAKE_PYTHONPATH
  "${DRAKE_PYTHONPATH}/python${drake_PYTHON_VERSION}/site-packages"
)

# (Optional) Copy the models directory to the binary directory so that runtime file paths match.
file(COPY ${CMAKE_SOURCE_DIR}/models DESTINATION ${CMAKE_BINARY_DIR})

# Add the subdirectory that contains the target definitions.
add_subdirectory(src)
