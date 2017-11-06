cmake_minimum_required(VERSION 2.8.3)
project(counter_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
  FILES
  counter.srv
)

generate_messages(
  DEPENDENCIES
  chatbot_node
  message_ui
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(counter_node src/counter_node.cpp)
add_dependencies(counter_node ${catkin_EXPORTED_TARGETS})
add_dependencies(counter_node message_ui_generate_messages_cpp)
add_dependencies(counter_node chatbot_node_generate_messages_cpp)
add_dependencies(counter_node arithmetic_node_generate_messages_cpp)
target_link_libraries(counter_node ${catkin_LIBRARIES} )
