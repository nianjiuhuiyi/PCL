# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
#
cmake_minimum_required (VERSION 3.8)

project ("PCL")

# 设置输出根目录为build/Debug
set(OUTPUT_DIRECTORY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/build/${CMAKE_BUILD_TYPE})
# 设置可执行程序输出到build/Debug/bin目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/bin" CACHE PATH "Runtime directory" FORCE)
# 设置库文件输出到build/Debug/lib目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Library directory" FORCE)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${OUTPUT_DIRECTORY_ROOT}/lib" CACHE PATH "Archive directory" FORCE)


# 包含子项目。
# add_subdirectory ("00BASIC")
# add_subdirectory ("01common")
# add_subdirectory ("02KdTree")
# add_subdirectory ("03Octree")
# add_subdirectory ("04filtering")
# add_subdirectory ("05RangeImage")
# add_subdirectory ("06Keypoints")
# add_subdirectory ("07RANSAC")
# add_subdirectory ("08normal")
# add_subdirectory ("09feature_pfh")
# add_subdirectory ("10getOBB")
# add_subdirectory ("11ICPANDNDT")
# add_subdirectory ("12segmentation")
add_subdirectory ("13surface")

