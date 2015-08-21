cmake_minimum_required(VERSION 2.8)
project(planar_manipulator)

find_package(catkin REQUIRED COMPONENTS
  rtt_ros rtt_rosclock std_msgs tf eigen_conversions cmake_modules collision_convex_model kin_dyn_model planer_utils
  )

find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(OMPL REQUIRED)

link_directories(${orocos_kdl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${USE_OROCOS_INCLUDE_DIRS} ${OMPL_INCLUDE_DIRS})

add_executable(planar_sim src/planar_sim.cpp src/planar5_dyn_model.cpp )
add_executable(planar_sim_planer_rrt src/planar_sim_planer_rrt.cpp src/planar5_dyn_model.cpp)
add_executable(test_ompl src/test_ompl.cpp)
add_executable(planar_rrt src/planar_rrt.cpp src/planar5_dyn_model.cpp src/rrt.cpp)

target_link_libraries(planar_sim ${catkin_LIBRARIES})
target_link_libraries(planar_sim_planer_rrt ${catkin_LIBRARIES})
target_link_libraries(test_ompl ${catkin_LIBRARIES} ${OMPL_LIBRARIES})
target_link_libraries(planar_rrt ${catkin_LIBRARIES})

catkin_package(
    DEPENDS ompl eigen
)

