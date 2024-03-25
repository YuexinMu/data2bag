cmake_minimum_required(VERSION 3.5)
project(data2bag)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(Threads REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(class_loader REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(OpenCV REQUIRED)
find_package(SDL2 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL COMPONENTS common REQUIRED)

set(dependencies
        OpenCV
        geometry_msgs
        visualization_msgs
        pcl_conversions
        rcl_interfaces
        rclcpp
        rcutils
        sensor_msgs
        std_msgs
        tf2
        tf2_ros
        tf2_msgs
        )

include_directories(
        include
        ${PCL_INCLUDE_DIRS}
)

set(data2bag_base src/data2bag_base.cpp)

add_library(${PROJECT_NAME} SHARED ${data2bag_base})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(data2bag_kitti src/data2bag_kitti.cpp)
ament_target_dependencies(data2bag_kitti ${dependencies})
target_link_libraries(data2bag_kitti
        ${PROJECT_NAME}
        ${Boost_SYSTEM_LIBRARY}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        )

install(DIRECTORY include/ DESTINATION include/)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS
        ${PROJECT_NAME}
        data2bag_kitti
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
