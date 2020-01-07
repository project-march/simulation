# 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(march_gazebo_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        gazebo_ros
        march_shared_resources
        roscpp
        std_msgs
        )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
        DEPENDS
        roscpp
        gazebo_ros
)

include_directories(
    ${roscpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/msgs
)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/msgs)
add_subdirectory(msgs)

add_library(${PROJECT_NAME} src/simple_world_plugin.cpp)
target_link_libraries(${PROJECT_NAME} march_gazebo_messages ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(${PROJECT_NAME}  march_gazebo_messages)

add_library(com_controller src/com_controller.cpp)
target_link_libraries(com_controller march_gazebo_messages ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(com_controller march_gazebo_messages)

install(DIRECTORY launch worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME} com_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(pub src/pub.cpp)

target_link_libraries(pub ${GAZEBO_LIBRARIES})

