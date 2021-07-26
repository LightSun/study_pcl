cmake_minimum_required(VERSION 3.5)

project(pcl1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(E:/study/cpp/cpp_builds/opencv/build/include)
link_directories(E:/study/cpp/cpp_builds/opencv/sources/.build/lib/Debug)
SET(OPENCV_LIBS opencv_core451d
    opencv_highgui451d
    opencv_imgcodecs451d
    opencv_imgproc451d
    opencv_videoio451d)

add_executable(pcl1
    testmain.h studyoctree.h
    testmain.cpp studyoctree.cpp
    study_sample_consensus.h study_sample_consensus.cpp
    study_range_image.h study_range_image.cpp
    study_fliter.h study_fliter.cpp
    studyio.h studyio.cpp
    study_key_point.h study_key_point.cpp
    study_features.h study_features.cpp
    study_segmentation.h study_segmentation.cpp
    study_registration.h study_registration.cpp
    volum_detect_viewer.h volum_detect_viewer.cpp
    main.cpp)

target_link_libraries (pcl1 ${PCL_LIBRARIES} ${OPENCV_LIBS})
