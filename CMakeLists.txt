cmake_minimum_required(VERSION 3.0.2)
project(xgc_session_clock_guard)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  rosgraph_msgs
  std_msgs
)

add_message_files(
  FILES
  ClockGuardAggregateStatus.msg
  ClockGuardEvent.msg
  ClockGuardStatus.msg
  ClockTimestampEnvelope.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xgc_session_clock_guard_core
  CATKIN_DEPENDS geometry_msgs message_runtime roscpp rosgraph_msgs std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(xgc_session_clock_guard_core
  src/core/clock_mapper.cpp
  src/core/epoch_fence.cpp
  src/core/frozen_config.cpp
  src/core/healthcheck.cpp
  src/core/session_clock_guard.cpp
  src/core/sha256.cpp
  src/core/types.cpp
)
target_compile_features(xgc_session_clock_guard_core PUBLIC cxx_std_17)
target_compile_options(xgc_session_clock_guard_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(session_clock_guard_node
  src/ros/session_clock_guard_node.cpp
)
add_executable(session_clock_guard_healthcheck
  src/ros/session_clock_guard_healthcheck.cpp
)
add_dependencies(session_clock_guard_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
add_dependencies(session_clock_guard_healthcheck
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_compile_features(session_clock_guard_node PUBLIC cxx_std_17)
target_compile_options(session_clock_guard_node PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(session_clock_guard_node
  xgc_session_clock_guard_core
  ${catkin_LIBRARIES}
)
target_compile_features(session_clock_guard_healthcheck PUBLIC cxx_std_17)
target_compile_options(session_clock_guard_healthcheck PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(session_clock_guard_healthcheck
  xgc_session_clock_guard_core
  ${catkin_LIBRARIES}
)

install(TARGETS xgc_session_clock_guard_core session_clock_guard_node session_clock_guard_healthcheck
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

install(DIRECTORY config docs launch runtime-manifests
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(session_clock_guard_catkin_tests test/core_catkin_tests.cpp)
  if(TARGET session_clock_guard_catkin_tests)
    target_compile_features(session_clock_guard_catkin_tests PUBLIC cxx_std_17)
    target_compile_options(session_clock_guard_catkin_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(session_clock_guard_catkin_tests xgc_session_clock_guard_core)
  endif()
endif()
