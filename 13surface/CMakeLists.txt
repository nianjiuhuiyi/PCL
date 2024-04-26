cmake_minimum_required (VERSION 3.8)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
# 添加预处理器和编译器标记
add_definitions(${PCL_DEFINITIONS})
add_definitions(-DPCL_NO_PRECOMPILE)

add_executable (pcl_demo "mls.cpp")
target_link_libraries(pcl_demo ${PCL_LIBRARIES})
set_target_properties(pcl_demo PROPERTIES COMPILE_FLAGS "/bigobj")