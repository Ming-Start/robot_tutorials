# cmake_minimum_required(VERSION 3.0.2)
# project(fg_gazebo_example)

# find_package(message_generation REQUIRED)

# add_message_files(
#   FILES
#   ObjectInfo.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   std_msgs
# )
# # find_package(catkin REQUIRED)
# catkin_package()

# catkin_install_python(PROGRAMS
#   scripts/move_viewpoints.py
#   scripts/target_reach.py
#   scripts/avoid_obstacle.py
#   scripts/bowl.py
#   scripts/1.py
#   scripts/object.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(DIRECTORY assets launch meshes urdf
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
cmake_minimum_required(VERSION 3.0.2)
project(fg_gazebo_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ObjectInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/move_viewpoints.py
  scripts/target_reach.py
  scripts/avoid_obstacle.py
  scripts/bowl.py
  scripts/1.py
  scripts/object.py
  scripts/avoid_object.py
  scripts/move_box.py
  scripts/avoid_move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY assets launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)