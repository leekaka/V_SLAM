#声明要求的 cmake 最低版本
cmake_minimum_required(  VERSION 2.8 )

# 设置运行模式
set( CMAKE_BUILD_TYPE "Debug")

# 声明一个cmake工程
project( HelloSLAM )

# 添加一个可执行程序
# 语法:add_executable (程序名 源代码文件)

add_executable( helloSlam helloSLAM.cpp )

# 添加一个库
add_library(hello_shared SHARED libHelloSlam.cpp)

# 链接一个库
add_executable( usehello useHello.cpp )
target_link_libraries( usehello hello_shared )
