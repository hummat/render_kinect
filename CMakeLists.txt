cmake_minimum_required(VERSION 2.4.6)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(CMAKE_BUILD_TYPE Release)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(PROJECT_NAME kinectSim)
#set the path for project includes
include_directories(${PROJECT_SOURCE_DIR}/include)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# There exist different versions of the assimp library for different 
# ubuntu distros. 
execute_process(COMMAND lsb_release -sc 
  OUTPUT_VARIABLE _distro OUTPUT_STRIP_TRAILING_WHITESPACE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_${_distro}")
message("Compiling for Ubuntu version ${_distro}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(CGAL REQUIRED)
include_directories(${CGAL_INCLUDE_DIRS})

find_package(OpenMP QUIET)
if(OPENMP_FOUND)
  message("Found OpenMP")
  include_directories(${OpenMP_INCLUDE_DIRS})
  set(LIBS ${LIBS} ${OpenMP_LIBRARIES})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -DHAVE_OMP")
endif(OPENMP_FOUND)

#find_package(OpenCV QUIET REQUIRED COMPONENTS core highgui PATHS /volume/USERSTORE/f_justin/epackages/opencv_3.4.18/install)
find_package(OpenCV QUIET REQUIRED COMPONENTS core highgui)
if(OpenCV_FOUND)
  message("Found OpenCV")
  include_directories(${OpenCV_INCLUDE_DIRS})
  set(OpenCV_LIBS ${OpenCV_LIBS} opencv_imgcodecs)
  set(LIBS ${LIBS} ${OpenCV_LIBS})
endif(OpenCV_FOUND)
#include_directories(/home/wink_do/PycharmProjects/render_kinect/assimp/install/include)
#include_directories(/home/wink_do/PycharmProjects/render_kinect/libnoise/install/include)


add_library(${PROJECT_NAME} SHARED src/kinectSimulator.cpp src/noiseutils.cpp src/main_kinect.cpp)
target_link_libraries(${PROJECT_NAME} assimp CGAL noise ${LIBS}) 
#target_link_directories(${PROJECT_NAME} PRIVATE /home/wink_do/PycharmProjects/render_kinect/libnoise/install/bin)
#target_link_directories(${PROJECT_NAME} PRIVATE /home/wink_do/PycharmProjects/render_kinect/assimp/install/lib)

add_executable(render_object src/main_kinect.cpp)
target_link_libraries(render_object ${PROJECT_NAME})
#target_link_directories(render_object PRIVATE /home/wink_do/PycharmProjects/render_kinect/libnoise/install/bin)
#target_link_directories(render_object PRIVATE /home/wink_do/PycharmProjects/render_kinect/assimp/install/lib)
