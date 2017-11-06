cmake_minimum_required(VERSION 2.8.3)
project(arithmetic_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   arithmetic_reply.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_compile_options(-std=c++11)
add_executable(arithmetic_node src/arithmetic_node.cpp)
add_dependencies(arithmetic_node ${catkin_EXPORTED_TARGETS})
add_dependencies(arithmetic_node message_ui_generate_messages_cpp)
target_link_libraries(arithmetic_node ${catkin_LIBRARIES} )
