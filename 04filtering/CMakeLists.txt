cmake_minimum_required (VERSION 3.8)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS})


add_executable (pcl_demo "filtering.cpp")
target_link_libraries(pcl_demo ${PCL_LIBRARIES})
