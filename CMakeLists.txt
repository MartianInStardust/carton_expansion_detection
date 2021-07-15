cmake_minimum_required(VERSION 3.0.0)
project(carton_expansion_detection VERSION 0.1.0)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
if(COMPILER_SUPPORTS_CXX14)
   message(STATUS "Using flag -std=c++14.")
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -std=c++14 -Wall -pthread -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall  -pthread -O3 -march=native")

# find_package(Threads REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV 4 REQUIRED)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.4.5 REQUIRED)
      if(NOT OpenCV_FOUND)
         message(FATAL_ERROR "OpenCV3.4 and OpenCV4 all not found.")
      endif()
endif()
message(STATUS "Opencv version:" ${OpenCV_VERSION})
include_directories(${OpenCV_INCLUDE_DIRS})

find_package( PCL 1.9 REQUIRED)
# find_package(PCL 1.9 REQUIRED REQUIRED COMPONENTS io registration segmentation visualization)
include_directories( ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions( ${PCL_DEFINITIONS} )

find_package (PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP REQUIRED yaml-cpp) # search yaml-cpp 
link_directories(${YAMLCPP_LIBRARY_DIRS}) # add yaml-cpp library path


include_directories(${PROJECT_SOURCE_DIR}/include
                     ${PROJECT_SOURCE_DIR}/include/percipio
                     ${PROJECT_SOURCE_DIR}/src/image_method
                     ${PROJECT_SOURCE_DIR}/example
                  )

### percipio 3D
set(PERCIPIO_LIB -ltycam)
set(PERCIPIO_COMMON_SOURCES ${PROJECT_SOURCE_DIR}/example/common/MatViewer.cpp ${PROJECT_SOURCE_DIR}/example/common/TYThread.cpp)

add_library(percipio_sample_common STATIC ${PERCIPIO_COMMON_SOURCES})
target_link_libraries(percipio_sample_common pthread)

### Azure Kinect
find_package(k4a REQUIRED)
find_package(k4arecord REQUIRED)

### cv utility library
add_library(cv_process STATIC 
            ${PROJECT_SOURCE_DIR}/src/image_method/cv_image_extraction.h
            ${PROJECT_SOURCE_DIR}/src/image_method/cv_image_extraction.cpp
            )
target_link_libraries(cv_process ${PERCIPIO_LIB} ${OpenCV_LIBS})

### pcl utility library
add_library(pcl_process STATIC
   ${PROJECT_SOURCE_DIR}/include/pcl_cluster_extraction.h
   ${PROJECT_SOURCE_DIR}/include/pcl_utility.h
   ${PROJECT_SOURCE_DIR}/src/pcl_cluster_extraction.cpp
   ${PROJECT_SOURCE_DIR}/src/pcl_utility.cpp
)
target_link_libraries(pcl_process  ${PCL_LIBRARIES} ${YAMLCPP_LIBRARIES})

add_executable(test_linelaser_detect example/test_linelaser_detect.cpp)
target_link_libraries(test_linelaser_detect
               pcl_process
               ${PCL_LIBRARIES} 
               ${YAMLCPP_LIBRARIES}
               )

add_executable(test_get_standard_normal example/test_get_standard_normal.cpp)
target_link_libraries(test_get_standard_normal
               pcl_process
               ${PCL_LIBRARIES} 
               ${YAMLCPP_LIBRARIES}
               )

 add_executable(test_percipio_rgbd_detect example/test_percipio_rgbd_detect.cpp)
target_link_libraries(test_percipio_rgbd_detect
                              percipio_sample_common
                              cv_process
                              pcl_process
                              ${PERCIPIO_LIB}
                              ${OpenCV_LIBS}
                              ${PCL_LIBRARIES} 
                              ${YAMLCPP_LIBRARIES}
                              )
### kinect library
# add_library(kinect_sample_common STATIC
#          ${PROJECT_SOURCE_DIR}/example/kinect_k4a/camera_data.h
#          ${PROJECT_SOURCE_DIR}/example/kinect_k4a/k4a_camera.cpp
#          ${PROJECT_SOURCE_DIR}/example/kinect_k4a/k4a_camera.h
#          )
# target_link_libraries(kinect_sample_common pthread)          

add_executable(test_kinect_rgbd_detect example/test_kinect_rgbd_detect.cpp example/kinect_k4a/k4a_camera.cpp)
target_include_directories(test_kinect_rgbd_detect PUBLIC example/kinect_k4a)
target_link_libraries(test_kinect_rgbd_detect
               k4a
               k4arecord
               cv_process
               pcl_process
               # kinect_sample_common
               ${OpenCV_LIBS}
               ${PCL_LIBRARIES} 
               ${YAMLCPP_LIBRARIES}
               )
