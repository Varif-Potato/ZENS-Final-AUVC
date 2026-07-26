# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target msgs::msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${msgs_TARGETS}.
if(msgs_TARGETS AND NOT TARGET msgs::msgs)
  add_library(msgs::msgs INTERFACE IMPORTED)
  set_target_properties(msgs::msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${msgs_TARGETS}")
endif()
