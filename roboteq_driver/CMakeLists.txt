cmake_minimum_required(VERSION 2.8.3)
project(roboteq_driver)

find_package(catkin REQUIRED COMPONENTS roboteq_msgs roscpp serial)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roboteq_msgs roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Build and install the roboteq_driver library.
add_subdirectory(src/${PROJECT_NAME})

# Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/driver.cpp)
add_dependencies(${PROJECT_NAME}_node roboteq_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME driver_node PREFIX "")

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

