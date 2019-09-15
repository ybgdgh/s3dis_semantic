cmake_minimum_required(VERSION 2.6)
project(s3dis)

set( CMAKE_BUILD_TYPE Debug )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# add_library( stereo_correct_shared SHARED stereo_correct.cpp )
# add_library(ORBextractor_shared SHARED ORBextractor.cpp)

# pcl 
find_package( PCL REQUIRED   )
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )
include_directories( ${PCL_LIBRARIES} )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


find_package(OpenCV 3 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories("/usr/include/eigen3")

add_executable(s3dis main.cpp)
target_link_libraries( s3dis  ${OpenCV_LIBS} ${PCL_LIBRARIES})




