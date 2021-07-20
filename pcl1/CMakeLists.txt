cmake_minimum_required(VERSION 3.5)

project(pcl1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl1
    testmain.h studyoctree.h
    testmain.cpp studyoctree.cpp
    study_sample_consensus.h study_sample_consensus.cpp
    main.cpp)

target_link_libraries (pcl1 ${PCL_LIBRARIES})
