cmake_minimum_required(VERSION 2.8)
project(planar_manipulator)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros rtt_rosclock std_msgs eigen_conversions cmake_modules image_transport self_collision
  )

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS})

add_executable(planar_sim src/planar_sim.cpp src/planar5_dyn_model.cpp src/fk_ik.cpp src/marker_publisher.cpp)
target_link_libraries(planar_sim ${catkin_LIBRARIES})

catkin_package()

