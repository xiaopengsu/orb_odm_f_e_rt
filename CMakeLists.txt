cmake_minimum_required( VERSION 2.8 )
project( vo1 )

#set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_BUILD_TYPE "Debug")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV 3.1 REQUIRED )
# find_package( OpenCV REQUIRED ) # use this if in OpenCV2
#set(G2O_ROOT /home/ubuntu/SLAM/slambook_master/3rdparty/g2o/g2o)


include_directories( 
    ${OpenCV_INCLUDE_DIRS}
    "/usr/include/eigen3/"
)

# add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp extra.cpp ) # use this if in OpenCV2 
add_executable( pose_estimation_2d2d pose_estimation_2d2d.cpp )
target_link_libraries( pose_estimation_2d2d ${OpenCV_LIBS} )
