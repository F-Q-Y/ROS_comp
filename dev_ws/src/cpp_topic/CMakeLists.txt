cmake_minimum_required(VERSION 3.5)
project(cpp_topic)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(SOURCES
src/atomic_op.h
src/atomic_op.h
src/bits_op.h
src/bits_op.c
src/def.h
src/extern_def.h
src/lock.c
src/mem.h 
src/mem.c
src/threads.h 
src/threads.c
src/topic_base.hpp
src/topic.cpp)

add_executable(topic
  ${SOURCES})

ament_target_dependencies(
  topic
  rclcpp
  std_msgs)

install(TARGETS
  topic
  DESTINATION lib/${PROJECT_NAME})

ament_package()
