cmake_minimum_required(VERSION 2.8.3)
project(rotors_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs mav_msgs roscpp sensor_msgs)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    mav_msgs
    roscpp
    sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(waypoint_publisher src/waypoint_publisher.cpp)
target_link_libraries(waypoint_publisher ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_publisher_file src/waypoint_publisher_file.cpp)
target_link_libraries(waypoint_publisher_file ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher_file ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example src/hovering_example.cpp)
target_link_libraries(hovering_example ${catkin_LIBRARIES})
add_dependencies(hovering_example ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example1 src/hovering_example1.cpp)
target_link_libraries(hovering_example1 ${catkin_LIBRARIES})
add_dependencies(hovering_example1 ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example2 src/hovering_example2.cpp)
target_link_libraries(hovering_example2 ${catkin_LIBRARIES})
add_dependencies(hovering_example2 ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example3 src/hovering_example3.cpp)
target_link_libraries(hovering_example3 ${catkin_LIBRARIES})
add_dependencies(hovering_example3 ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example4 src/hovering_example4.cpp)
target_link_libraries(hovering_example4 ${catkin_LIBRARIES})
add_dependencies(hovering_example4 ${catkin_EXPORTED_TARGETS})

add_executable(hovering_example5 src/hovering_example5.cpp)
target_link_libraries(hovering_example5 ${catkin_LIBRARIES})
add_dependencies(hovering_example5 ${catkin_EXPORTED_TARGETS})

foreach(dir launch models resource worlds)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

install(TARGETS waypoint_publisher waypoint_publisher_file hovering_example hovering_example1 hovering_example2 hovering_example3 hovering_example4 hovering_example5
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
