cmake_minimum_required(VERSION 3.0.0)
project(main VERSION 0.1.0)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ShowPCD main_ShowPCD.cpp)
target_link_libraries(ShowPCD ${PCL_LIBRARIES})

add_executable(VoxelPCD main_VoxelPCD.cpp)
target_link_libraries(VoxelPCD ${PCL_LIBRARIES})
