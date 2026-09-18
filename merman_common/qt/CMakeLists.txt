cmake_minimum_required(VERSION 3.16)
project(merman_chassis_panel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(merman_chassis_panel
    src/main.cpp
    src/chassis_panel.cpp
    src/map_widget.cpp
)

target_include_directories(merman_chassis_panel PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
)

target_link_libraries(merman_chassis_panel PRIVATE
    Qt5::Widgets
    rclcpp::rclcpp
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${sensor_msgs_LIBRARIES}
    ${std_srvs_LIBRARIES}
)

add_custom_command(TARGET merman_chassis_panel POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_SOURCE_DIR}/../bin
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:merman_chassis_panel>
        ${CMAKE_CURRENT_SOURCE_DIR}/../bin/
)
