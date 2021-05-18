cmake_minimum_required(VERSION 2.8)
project(semantic_msgs)

set(PACKAGE_DEPENDENCIES
    message_generation
    message_runtime
    std_msgs
    sensor_msgs
)

find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

add_message_files(DIRECTORY msg
    FILES
    SemanticLabel.msg
    SemanticPointCloud2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Make modules in include/ visible to work place
# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES} message_runtime
)
