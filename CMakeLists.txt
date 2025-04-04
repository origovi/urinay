cmake_minimum_required(VERSION 3.0.2)
project(urinay)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ofast")
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs custom_msgs sensor_msgs roslib eigen_conversions)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_exec src/main.cpp)
add_library(${PROJECT_NAME}
  src/structures/Point.cpp
  src/structures/Node.cpp
  src/structures/Edge.cpp
  src/structures/Circle.cpp
  src/structures/Triangle.cpp
  src/structures/Trace.cpp
  src/structures/TraceBuffer.cpp
  src/structures/Vector.cpp
  src/modules/DelaunayTri.cpp
  src/modules/WayComputer.cpp
  src/modules/Visualization.cpp
  src/utils/Time.cpp
  src/utils/Params.cpp
  src/utils/Failsafe.cpp
  src/utils/KDTree.cpp
)

add_dependencies(${PROJECT_NAME}_exec custom_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} custom_msgs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME}_exec ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)