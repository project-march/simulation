cmake_minimum_required(VERSION 2.8FATAL_ERROR)
project(march_gazebo_plugins)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
        march_shared_resources
        )
find_package(gazebo REQUIRED)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(com_controller src/com_controller.cpp)
target_link_libraries(com_controller ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(TARGETS com_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )
