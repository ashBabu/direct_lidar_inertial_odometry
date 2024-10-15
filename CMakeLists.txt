cmake_minimum_required(VERSION 3.8)
project(lidar_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/SavePCD.srv
    DEPENDENCIES       
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)
include_directories(SYSTEM)

# NanoFLANN
add_library(nanoflann STATIC
        src/nano_gicp/nanoflann.cc
)
target_link_libraries(nanoflann ${PCL_LIBRARIES})
ament_target_dependencies(nanoflann)

# NanoGICP
add_library(nano_gicp STATIC
        src/nano_gicp/lsq_registration.cc
        src/nano_gicp/nano_gicp.cc
)
target_link_libraries(nano_gicp ${PCL_LIBRARIES} ${OpenMP_LIBS} nanoflann)
ament_target_dependencies(nano_gicp)


# Odometry Node
add_executable(lm_odom_node src/lidar_mapping/odom_node.cc src/lidar_mapping/odom.cc)
target_link_libraries(lm_odom_node ${PCL_LIBRARIES} ${OpenMP_LIBS} Threads::Threads nano_gicp)
ament_target_dependencies(lm_odom_node "tf2_ros" "pcl_ros" "nav_msgs" "pcl_conversions" "rclcpp" "sensor_msgs" "geometry_msgs")

# Mapping Node
add_executable (lm_map_node src/lidar_mapping/map_node.cc src/lidar_mapping/map.cc)
target_link_libraries(lm_map_node ${PCL_LIBRARIES}  ${cpp_typesupport_target})
ament_target_dependencies(lm_map_node "tf2_ros" "pcl_ros" "pcl_conversions" "rclcpp")

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} nano_gicp nanoflann)
ament_export_dependencies(rclcpp std_msgs sensor_msgs geometry_msgs pcl_ros)
ament_package()

# Binaries
install( TARGETS lm_odom_node lm_map_node
         DESTINATION lib/${PROJECT_NAME} 
)
install( DIRECTORY config launch rviz
         DESTINATION share/${PROJECT_NAME} 
)

add_compile_options(-std=c++14)
link_directories(${PCL_LIBRARY_DIRS})

include(FindOpenMP)
if(OPENMP_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
else(OPENMP_FOUND)
  message("ERROR: OpenMP could not be found.")
endif(OPENMP_FOUND)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

target_compile_options(lm_odom_node PRIVATE ${OpenMP_FLAGS})

target_compile_options(lm_map_node PRIVATE ${OpenMP_FLAGS})

#ament_package()
