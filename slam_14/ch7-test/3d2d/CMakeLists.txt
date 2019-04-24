cmake_minimum_required( VERSION 2.8 )
project( pose_estimation_3d2d )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# head file path
INCLUDE_DIRECTORIES(
    include
    )

# source directory
AUX_SOURCE_DIRECTORY(src DIR_SRCS)


# 添加cmake模块以使用g2o
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

find_package( OpenCV 3.1 REQUIRED )
# find_package( OpenCV REQUIRED ) # use this if in OpenCV2 
find_package( G2O REQUIRED )
find_package( CSparse REQUIRED )

include_directories( 
    ${OpenCV_INCLUDE_DIRS} 
    ${G2O_INCLUDE_DIRS}
    ${CSPARSE_INCLUDE_DIR}
    "/usr/include/eigen3/"
)

# set environment cariable
SET(SRC_TEST ${DIR_SRCS})


#add_executable( feature_extraction ${DIR_SRCS}/orb_feature.cpp  )
#target_link_libraries( feature_extraction ${OpenCV_LIBS} )

add_executable( ${PROJECT_NAME} ${SRC_TEST})
target_link_libraries( ${PROJECT_NAME}  ${OpenCV_LIBS})
