# 声明要求的cmake的最低版本
cmake_minimum_required( VERSION 2.6 )

# 声明一个工程
project( TestSophus )


# 为使用 sophus，您需要使用find_package命令找到它
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )

add_executable( useSophus useSophus.cpp )
target_link_libraries( useSophus ${Sophus_LIBRARIES} )


