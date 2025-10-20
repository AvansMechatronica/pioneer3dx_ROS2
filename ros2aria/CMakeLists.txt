cmake_minimum_required(VERSION 3.8)
project(ros2aria)

# --- Dependencies ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros2aria_interfaces REQUIRED)


# --- Optional Aria dependency ---
find_package(Aria QUIET)
if(Aria_FOUND)
  message(STATUS "Found Aria via CMake package.")
  include_directories(${Aria_INCLUDE_DIRS})
  link_directories(${Aria_LIBRARY_DIRS})
  add_definitions(-DADEPT_PKG)
else()
  if(DEFINED ENV{ARIA})
    set(ARIA_PREFIX $ENV{ARIA})
  else()
    set(ARIA_PREFIX "/usr/local")
  endif()

  # added by gerard above does not work
  set(ARIA_PREFIX "/usr/local")


  message(STATUS "Looking for Aria in ${ARIA_PREFIX}...")

  if(EXISTS "${ARIA_PREFIX}/Aria/include/Aria.h")
    message(STATUS "Found Adept ARIA at ${ARIA_PREFIX}/Aria/include")
    include_directories(${ARIA_PREFIX}/Aria/include)
    link_directories(${ARIA_PREFIX}/Aria/lib)
    add_definitions(-DADEPT_PKG)
  elseif(EXISTS "${ARIA_PREFIX}/include/Aria.h")
    message(STATUS "Found Adept ARIA headers in ${ARIA_PREFIX}/include")
    include_directories(${ARIA_PREFIX}/include)
    link_directories(${ARIA_PREFIX}/lib)
  elseif(EXISTS "${ARIA_PREFIX}/include/Aria/Aria.h")
    message(STATUS "Found AriaCoda-style headers in ${ARIA_PREFIX}/include/Aria")
    include_directories(${ARIA_PREFIX}/include)
    link_directories(${ARIA_PREFIX}/lib)
  else()
    message(WARNING "Aria.h not found; continuing with default paths.")
  endif()
endif()


# --- Include dirs ---
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# --- Source files ---
set(SOURCES
  src/LaserPublisher.cpp
  src/Ros2Aria.cpp
)

# --- Build executable ---
add_executable(ros2aria_node ${SOURCES})

target_link_libraries(ros2aria_node
  ${Boost_LIBRARIES}
  Aria
  pthread
  dl
  rt
)

ament_target_dependencies(ros2aria_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  std_srvs
  ros2aria_interfaces
)

# Install launch files
#install(
#  DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)

# --- Install ---
install(
  TARGETS ros2aria_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# --- Export package ---
ament_package()
