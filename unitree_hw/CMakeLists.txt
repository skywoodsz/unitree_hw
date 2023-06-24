cmake_minimum_required(VERSION 3.0.2)
project(unitree_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        controller_manager
        cheetah_msgs
        )

find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        controller_manager
        cheetah_msgs
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

link_directories(lib)
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)

add_executable(${PROJECT_NAME}
        src/unitree_hw_node.cpp
        src/UnitreeHw.cpp
        src/HwLoop.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${EXTRA_LIBS}
        )

