cmake_minimum_required(VERSION 3.10)
project(kinect_ext)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Output directories
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Project includes
include_directories(${PROJECT_SOURCE_DIR}/include)

# Bundled libnoise
include_directories(${PROJECT_SOURCE_DIR}/lib/libnoise/include)
link_directories(${PROJECT_SOURCE_DIR}/lib/libnoise/lib)

# Ubuntu version detection (for assimp header compatibility)
execute_process(COMMAND lsb_release -sc OUTPUT_VARIABLE _distro OUTPUT_STRIP_TRAILING_WHITESPACE)
if(_distro)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_${_distro}")
  message(STATUS "Detected distro: ${_distro}")
endif()

# Required dependencies
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 found: ${EIGEN3_INCLUDE_DIR}")

find_package(CGAL REQUIRED)
message(STATUS "CGAL found: ${CGAL_INCLUDE_DIRS}")

find_package(OpenCV REQUIRED COMPONENTS core highgui)
message(STATUS "OpenCV found: ${OpenCV_VERSION}")
include_directories(${OpenCV_INCLUDE_DIRS})
set(OpenCV_LIBS ${OpenCV_LIBS} opencv_imgcodecs)

# Optional dependencies
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  message(STATUS "OpenMP found - enabling parallelization")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_OMP")
  set(LIBS ${LIBS} ${OpenMP_LIBRARIES})
endif()

find_package(PCL 1.3 QUIET COMPONENTS io)
if(PCL_FOUND)
  message(STATUS "PCL found - enabling point cloud output")
  include_directories(${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_PCL")
  set(LIBS ${LIBS} ${PCL_LIBRARIES})
else()
  message(STATUS "PCL not found - point cloud output disabled")
endif()

# Main library (static to avoid libnoise PIC issues)
add_library(${PROJECT_NAME} STATIC
  src/kinectSimulator.cpp
  src/noiseutils.cpp
  src/simulate.cpp
)
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  CGAL
  assimp
  noise
  noiseutils-static
  ${OpenCV_LIBS}
  ${LIBS}
)

# Test executable
add_executable(render_object src/main.cpp)
target_link_libraries(render_object ${PROJECT_NAME})
