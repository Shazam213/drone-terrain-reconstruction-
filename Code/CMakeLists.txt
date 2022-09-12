cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(voxel-pca-greedy)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable (voxel-pca-greedy VGF-PCA-GA.cpp)

target_link_libraries (voxel-pca-greedy ${PCL_LIBRARIES})
