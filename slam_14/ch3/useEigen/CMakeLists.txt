# 声明要求的cmake的最低版本
cmake_minimum_required( VERSION 2.6 )

# 声明一个工程
project( TestEigen )


# 添加一个可执行程序
ADD_EXECUTABLE(testEigen eigenMatrix.cpp)

ADD_EXECUTABLE(testGeometry useGeometry.cpp )
# 添加头文件
INCLUDE_DIRECTORIES( "/usr/include/eigen3" )

