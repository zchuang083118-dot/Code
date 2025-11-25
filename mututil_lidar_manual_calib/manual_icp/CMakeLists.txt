cmake_minimum_required(VERSION 3.5)
project(manual_icp)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 PCL
find_package(PCL 1.10 REQUIRED)

# 包含头文件目录
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 添加可执行文件
add_executable(manual_icp manual_icp.cpp)

# 链接 PCL 库
target_link_libraries(manual_icp ${PCL_LIBRARIES})
