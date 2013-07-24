cmake_minimum_required(VERSION 2.8.3)
project(kinfu)

find_package(catkin REQUIRED
    COMPONENTS message_generation roscpp std_srvs std_msgs sensor_msgs dynamic_reconfigure tf image_transport)

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#find_package(VTK REQUIRED)
#include_directories(${VTK_INCLUDE_DIRS})

find_package(PCL 1.7.0 REQUIRED COMPONENTS gpu_kinfu_large_scale)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/dyn_config.cfg)
include_directories(cfg/cpp)

#### Cuda 
find_package(CUDA)

if (CUDA_FOUND) 
    message(" * CUDA ${CUDA_VERSION} was found")
    include(FindCUDA)
    include_directories(${CUDA_INCLUDE_DIRS})
else(CUDA_FOUND)
    message(" * CUDA was not found")
    message(FATAL_ERROR "Not all CUDA libraries have been found")
endif(CUDA_FOUND)

# workaround for PCL src dependency!
if (NOT DEFINED PCL_SRC OR NOT EXISTS "${PCL_SRC}/package.xml")
    message(FATAL_ERROR "PCL_SRC has to point to the package 'pcl'.") 
endif (NOT DEFINED PCL_SRC OR NOT EXISTS "${PCL_SRC}/package.xml")

include_directories(${PCL_SRC}/pcl_trunk/src/gpu/kinfu_large_scale/src)

catkin_package(
   CATKIN_DEPENDS message_runtime std_msgs sensor_msgs dynamic_reconfigure
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

add_executable(kinfuLS src/kinfu.cpp)
add_dependencies(kinfuLS ${PROJECT_NAME}_gencfg pcl)

target_link_libraries(kinfuLS boost_system boost_signals)
target_link_libraries(kinfuLS ${catkin_LIBRARIES})

target_link_libraries(kinfuLS ${CUDA_LIBRARIES})
target_link_libraries(kinfuLS ${PCL_LIBRARIES})
