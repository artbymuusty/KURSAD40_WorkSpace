cmake_minimum_required(VERSION 3.10)
project(payload_drop_system)

find_package(gz-sim REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)

add_library(payload_drop_system SHARED payload_drop_system.cc)
target_link_libraries(payload_drop_system
  gz-sim::gz-sim
  gz-transport::gz-transport
  gz-msgs::gz-msgs
)

set_target_properties(payload_drop_system PROPERTIES OUTPUT_NAME "payload_drop_system")
