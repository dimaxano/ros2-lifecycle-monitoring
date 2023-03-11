cmake_minimum_required(VERSION 3.8)
project(ros2-lifecycle-monitoring)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Test Concurrent)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(cmr_clients_utils REQUIRED)

include_directories(include)

set(rviz_lifecycle_plugin_headers_to_moc
    include/ros2-lifecycle-monitoring/rviz_lifecycle_plugin.h)

set(library_name "rviz_lifecycle_plugin")

add_library(${library_name} SHARED
    src/rviz_lifecycle_plugin.cpp
    ${rviz_lifecycle_plugin_headers_to_moc})

set(dependencies
    pluginlib
    Qt5
    rclcpp
    rviz_common
    rviz_default_plugins
    rviz_rendering
    cmr_clients_utils
)

ament_target_dependencies(${library_name}
    ${dependencies})

target_include_directories(${library_name} PUBLIC
    ${Qt5Widgets_INCLUDE_DIRS})

target_link_libraries(${library_name}
    rviz_common::rviz_common)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
# TODO: Make this specific to this project (not rviz default plugins)
target_compile_definitions(${library_name} PRIVATE "RVIZ_DEFAULT_PLUGINS_BUILDING_LIBRARY")

# prevent pluginlib from using boost
target_compile_definitions(${library_name} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS ${library_name}
    EXPORT ${library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

install(DIRECTORY include/
    DESTINATION include/)

install(DIRECTORY
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    # set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    # set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_targets(${library_name} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    Qt5
    rviz_common
    rclcpp
)

ament_package()