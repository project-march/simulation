cmake_minimum_required(VERSION 2.8.3)
project(march_gazebo_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_ros
        )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
        DEPENDS
        roscpp
        gazebo_ros
)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/simple_world_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(com_controller src/com_controller.cpp)
target_link_libraries(com_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY launch worlds
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(TARGETS ${PROJECT_NAME} com_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
