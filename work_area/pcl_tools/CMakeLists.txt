cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(PCL_TOOLS)

find_package(PCL 1.3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIB_DIR})

add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_write_test pcd_write.cpp)
target_link_libraries(pcd_write_test ${PCL_LIBRARIES})
target_link_libraries(pcd_write_test ${OpenCV_LIBS} )

add_executable(stereo2PC_generic stereo2PC_generic.cpp)
target_link_libraries(stereo2PC_generic ${PCL_LIBRARIES})
target_link_libraries(stereo2PC_generic ${OpenCV_LIBS} )

add_executable(depth2PC_generic depth2PC_generic.cpp)
target_link_libraries(depth2PC_generic ${PCL_LIBRARIES})
target_link_libraries(depth2PC_generic ${OpenCV_LIBS} )

add_executable(depth2PC_tum depth2PC_tum.cpp)
target_link_libraries(depth2PC_tum ${PCL_LIBRARIES})
target_link_libraries(depth2PC_tum ${OpenCV_LIBS} )
