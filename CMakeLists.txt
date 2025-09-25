cmake_minimum_required(VERSION 3.16.3)

project(Climber_Vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})


add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
# add_subdirectory(tasks/omniperception)

####################src###################


##################calibration################


##################tests##################
add_executable(auto_aim_test test/auto_aim_test.cpp)
target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
add_executable(auto_buff_test test/auto_buff_test.cpp)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
# # 检查 ROS 环境
# find_package(ament_cmake QUIET)
# find_package(rclcpp QUIET)
# find_package(std_msgs QUIET)
# find_package(rosidl_typesupport_cpp QUIET)
# find_package(sp_msgs QUIET)
# # 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
# if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
#     # 编译 sentry 和 publish_test
#     add_executable(sentry src/sentry.cpp)
#     add_executable(sentry_bp src/sentry_bp.cpp)
#     add_executable(sentry_debug src/sentry_debug.cpp)
#     add_executable(sentry_multithread src/sentry_multithread.cpp)
#     add_executable(publish_test tests/publish_test.cpp)
#     add_executable(subscribe_test tests/subscribe_test.cpp)
#     add_executable(topic_loop_test tests/topic_loop_test.cpp)

#     target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
#     target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
#     target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
#     target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
#     target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
#     target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
#     target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
# else()
#     message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
# endif()
